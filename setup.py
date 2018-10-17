#!/usr/bin/env python
# -*- coding: utf-8 -*-
import glob
import io
import os
import subprocess
from os.path import join
import sys
import platform
from shutil import rmtree
from time import time
from collections import defaultdict

import numpy

import setuptools
from setuptools.command.build_ext import build_ext
from setuptools import find_packages, setup, Command, Extension
from distutils.errors import CompileError, DistutilsExecError

# Package meta-data.
NAME = 'pclpy'
DESCRIPTION = 'Python bindings for the Point Cloud Library'
URL = 'https://www.github.com/davidcaron/pclpy'
EMAIL = 'dcaron05@gmail.com'
AUTHOR = 'David Caron'
REQUIRES_PYTHON = '==3.6.*'
VERSION = None

PCL_VERSION = "1.8"
PCL_ROOT = os.getenv("PCL_ROOT")
PYTHON_HOME = os.path.split(sys.executable)[0]

ON_WINDOWS = platform.system() == "Windows"

REQUIRED = [
    'laspy',
    'numpy',
]

if ON_WINDOWS:
    REQUIRED.append('pclpy_dependencies>=0.2.1')

HERE = os.path.abspath(os.path.dirname(__file__))

DEBUG = False
if "--debug" in sys.argv:
    sys.argv.remove("--debug")
    DEBUG = True

if ON_WINDOWS:
    import distutils._msvccompiler

    # monkey-patch msvc to build with x64 compiler
    # this is necessary because ram exceeds 4Gb while compiling for some modules
    old = distutils._msvccompiler._get_vc_env
    distutils._msvccompiler._get_vc_env = lambda _: old("amd64")

    # temporary fix
    # can't find C:\Program Files (x86)\Windows Kits\10\bin\x64\rc.exe
    # for permanent solution see
    # https://stackoverflow.com/questions/43847542/rc-exe-no-longer-found-in-vs-2015-command-prompt
    os.environ["PATH"] += r";C:\Program Files (x86)\Windows Kits\10\bin\10.0.16299.0\x64"

    # For MSVC, this flag enables multiprocess compilation
    MSVC_MP_BUILD = False
    N_WORKERS = 4
    if "--msvc-mp-build" in sys.argv:
        sys.argv.remove("--msvc-mp-build")
        MSVC_MP_BUILD = True

    USE_CLCACHE = False
    if "--use-clcache" in sys.argv:
        sys.argv.remove("--use-clcache")
        USE_CLCACHE = True

        # ensure clcache exists
        for path in os.environ["PATH"].split(os.pathsep):
            if os.path.isfile(os.path.join(path, "clcache.exe")):
                break
        else:
            raise FileNotFoundError("You specified --use-clcache but clcache.exe can't be found.")

    # For MSVC, this flag skips code generation at linking
    # Do not set for release builds because some optimizations are skipped
    MSVC_NO_CODE_LINK = False
    if "--msvc-no-code-link" in sys.argv:
        sys.argv.remove("--msvc-no-code-link")
        MSVC_NO_CODE_LINK = True

with io.open(join(HERE, 'README.md'), encoding='utf-8') as f:
    long_description = '\n' + f.read()

# Load the package's __version__.py module as a dictionary.
about = {}
if not VERSION:
    with open(join(HERE, NAME, '__version__.py')) as f:
        exec(f.read(), about)
else:
    about['__version__'] = VERSION


class get_pybind_include(object):
    """Helper class to determine the pybind11 include path
    The purpose of this class is to postpone importing pybind11
    until it is actually installed, so that the ``get_include()``
    method can be invoked. """

    def __init__(self, user=False):
        self.user = user

    def __str__(self):
        import pybind11
        return pybind11.get_include(self.user)


# As of Python 3.6, CCompiler has a `has_flag` method.
# cf http://bugs.python.org/issue26689
def has_flag(compiler, flagname):
    """Return a boolean indicating whether a flag name is supported on
    the specified compiler.
    """
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.cpp') as f:
        f.write('int main (int argc, char **argv) { return 0; }')
        try:
            compiler.compile([f.name], extra_postargs=[flagname])
        except setuptools.distutils.errors.CompileError:
            return False
    return True


def cpp_flag(compiler):
    """Return the -std=c++[11/14] compiler flag.
    The c++14 is prefered over c++11 (when it is available).
    """
    if has_flag(compiler, '-std=c++14'):
        return '-std=c++14'
    elif has_flag(compiler, '-std=c++11'):
        return '-std=c++11'
    else:
        raise RuntimeError('Unsupported compiler -- at least C++11 support '
                           'is needed!')


class BuildExt(build_ext):
    """A custom build extension for adding compiler-specific options."""
    c_opts = {
        'msvc': ['/EHsc', "/openmp"],
        'unix': [],
    }
    if ON_WINDOWS and MSVC_NO_CODE_LINK:
        c_opts['msvc'].append("/bigobj")
    if sys.platform == 'darwin':
        c_opts['unix'] += ['-stdlib=libc++', '-mmacosx-version-min=10.7']

    def build_extensions(self):
        ct = self.compiler.compiler_type
        opts = self.c_opts.get(ct, [])
        if ct == 'unix':
            opts.append('-DVERSION_INFO="%s"' % self.distribution.get_version())
            opts.append(cpp_flag(self.compiler))
            if has_flag(self.compiler, '-fvisibility=hidden'):
                opts.append('-fvisibility=hidden')
        elif ct == 'msvc':
            opts.append('/DVERSION_INFO=\\"%s\\"' % self.distribution.get_version())
        for ext in self.extensions:
            ext.extra_compile_args = opts

        build_ext.build_extensions(self)


ext_args = defaultdict(list)
ext_args['include_dirs'].append(numpy.get_include())

if ON_WINDOWS:
    def compile(self, sources,
                output_dir=None, macros=None, include_dirs=None, debug=0,
                extra_preargs=None, extra_postargs=None, depends=None):
        if not self.initialized:
            self.initialize()
        compile_info = self._setup_compile(output_dir, macros, include_dirs,
                                           sources, depends, extra_postargs)
        macros, objects, extra_postargs, pp_opts, build = compile_info

        if MSVC_NO_CODE_LINK:
            self.ldflags_shared.remove("/LTCG")
            self.compile_options.remove("/GL")

        if USE_CLCACHE:
            self.cc = "clcache.exe"

        compile_opts = extra_preargs or []
        compile_opts.append('/c')
        if DEBUG:
            compile_opts.extend(self.compile_options_debug)
            if MSVC_MP_BUILD:
                compile_opts.extend(["/FS"])  # Force Synchronous PDB Writes
        else:
            compile_opts.extend(self.compile_options)

        add_cpp_opts = False

        if MSVC_MP_BUILD:
            workers = N_WORKERS
            from multiprocessing.pool import ThreadPool
            t = ThreadPool(workers)
            for obj in objects:
                t.apply_async(self.compile_single,
                              [obj, build, debug, pp_opts, compile_opts, add_cpp_opts, extra_postargs])
            t.close()
            t.join()
        else:
            for obj in objects:
                self.compile_single(obj, build, debug, pp_opts, compile_opts, add_cpp_opts, extra_postargs)

        return objects


    def compile_single(self, obj, build, debug, pp_opts, compile_opts, add_cpp_opts, extra_postargs):
        try:
            src, ext = build[obj]
        except KeyError:
            return
        if debug:
            # pass the full pathname to MSVC in debug mode,
            # this allows the debugger to find the source file
            # without asking the user to browse for it
            src = os.path.abspath(src)

        if ext in self._c_extensions:
            input_opt = "/Tc" + src
        elif ext in self._cpp_extensions:
            input_opt = "/Tp" + src
            add_cpp_opts = True
        elif ext in self._rc_extensions:
            # compile .RC to .RES file
            input_opt = src
            output_opt = "/fo" + obj
            try:
                self.spawn([self.rc] + pp_opts + [output_opt, input_opt])
            except DistutilsExecError as msg:
                raise CompileError(msg)
            return
        elif ext in self._mc_extensions:
            # Compile .MC to .RC file to .RES file.
            #   * '-h dir' specifies the directory for the
            #     generated include file
            #   * '-r dir' specifies the target directory of the
            #     generated RC file and the binary message resource
            #     it includes
            #
            # For now (since there are no options to change this),
            # we use the source-directory for the include file and
            # the build directory for the RC file and message
            # resources. This works at least for win32all.
            h_dir = os.path.dirname(src)
            rc_dir = os.path.dirname(obj)
            try:
                # first compile .MC to .RC and .H file
                self.spawn([self.mc, '-h', h_dir, '-r', rc_dir, src])
                base, _ = os.path.splitext(os.path.basename(src))
                rc_file = join(rc_dir, base + '.rc')
                # then compile .RC to .RES file
                self.spawn([self.rc, "/fo" + obj, rc_file])

            except DistutilsExecError as msg:
                raise CompileError(msg)
            return
        else:
            # how to handle this file?
            raise CompileError("Don't know how to compile {} to {}"
                               .format(src, obj))

        args = [self.cc] + compile_opts + pp_opts
        if add_cpp_opts:
            args.append('/EHsc')
        args.append(input_opt)
        args.append("/Fo" + obj)
        args.extend(extra_postargs)

        try:
            self.spawn(args)
        except DistutilsExecError as msg:
            raise CompileError(msg)


    # monkey-patch msvc compiler for faster windows builds
    from distutils._msvccompiler import MSVCCompiler

    MSVCCompiler.compile = compile
    MSVCCompiler.compile_single = compile_single

    INCLUDE_WINDOWS = join(PYTHON_HOME, r"Library", "include")
    LIB_DIR_WINDOWS = join(PYTHON_HOME, r"Library", "lib")

    inc_dirs = [join(PCL_ROOT, "include", "pcl-%s" % PCL_VERSION),
                join(PCL_ROOT, "3rdParty", "Eigen", "eigen3"),
                join(PCL_ROOT, "3rdParty", "Boost", "include", "boost-1_64"),
                join(PCL_ROOT, "3rdParty", "FLANN", "include"),
                join(PCL_ROOT, "3rdParty", "Qhull", "include"),
                join(INCLUDE_WINDOWS, "vtk-8.1"),
                INCLUDE_WINDOWS,
                ]

    ext_args['include_dirs'] += inc_dirs

    lib_dirs = [join(PCL_ROOT, "lib"),
                join(PCL_ROOT, "3rdParty", "Boost", "lib"),
                join(PCL_ROOT, "3rdParty", "FLANN", "lib"),
                join(PCL_ROOT, "3rdParty", "Qhull", "lib"),
                LIB_DIR_WINDOWS,
                ]

    ext_args['library_dirs'] += lib_dirs

    for lib in os.listdir(join(PCL_ROOT, "lib")):
        endswith = "debug.lib" if DEBUG else "release.lib"
        if lib.endswith(endswith):
            ext_args['libraries'].append(lib[:-4])

    win_libreleases = ['kernel32', 'user32', 'gdi32', 'winspool', 'comdlg32',
                       'advapi32', 'shell32', 'ole32', 'oleaut32',
                       'uuid', 'odbc32', 'odbccp32']
    ext_args['libraries'] += win_libreleases

    # for lib in os.listdir(join(PCL_ROOT, "3rdParty", "Boost", "lib")):
    #     if lib.endswith("vc140-mt-1_64.lib"):
    #         ext_args['libraries'].append(lib[:-4])

    for lib in os.listdir(LIB_DIR_WINDOWS):
        endswith = "D-8.1.lib" if DEBUG else "-8.1.lib"
        if lib.endswith(endswith):
            ext_args['libraries'].append(lib[:-4])

    win_opengl_libdirs = ['C:\\Program Files (x86)\\Microsoft SDKs\\Windows\\v8.1A\\Lib\\x64']

    ext_args['library_dirs'] += win_opengl_libdirs

    win_opengl_libreleases = ['OpenGL32']
    ext_args['libraries'] += win_opengl_libreleases


else:  # not Windows
    # to install:
    # libpcl-dev

    libs_to_build = ['common', 'features', 'filters', 'geometry', 'io', 'kdtree', 'keypoints', 'octree',
                     'recognition', 'sample_consensus', 'search', 'segmentation', 'stereo', 'surface',
                     'tracking', 'visualization']

    pcl_libraries = ["pcl_%s-%s" % (lib, PCL_VERSION) for lib in libs_to_build]

    def pkg_config_multi(arg, libs):
        output = []
        for lib in libs:  # 'ni'?
            for value in pkg_config(arg, lib):
                output.append(value[2:])
        return list(set(output))

    def pkg_config(arg, lib):
        command = ["pkg-config", arg, lib]
        output = subprocess.check_output(command).decode().strip()
        return output.split()

    def find_include(folder, include):
        return glob.glob(os.path.join(folder, include))[-1]

    # inc_dirs = [
    #     join(PCL_ROOT, "include", "pcl-%s" % PCL_VERSION),
    # ]
    inc_dirs = pkg_config_multi('--cflags-only-I', pcl_libraries)

    # vtk
    inc_dirs.append(os.getenv("VTK_INCLUDE_DIR", find_include("/usr/include", "vtk-*")))

    ext_args['include_dirs'] += inc_dirs
    ext_args['extra_compile_args '] += ['-Wunused-local-typedefs']

defines = [('EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET', '1')]
ext_args['define_macros'] += defines

ext_args['include_dirs'].append(get_pybind_include())
ext_args['include_dirs'].append(get_pybind_include(user=True))

src_path = join("pclpy", "src")
ext_args['include_dirs'].append(src_path)

modules_path = join(src_path, "generated_modules")
loaders = [join(modules_path, f) for f in os.listdir(modules_path)
           if f.endswith("_loader.cpp") and not f == "__main_loader.cpp"]

ext_modules = [
    Extension(
        'pclpy.pcl',
        ['pclpy/src/pclpy.cpp'] + loaders,
        language='c++',
        **ext_args
    ),
]

t = time()
setup(
    name=NAME,
    version=about['__version__'],
    description=DESCRIPTION,
    long_description=long_description,
    long_description_content_type='text/markdown',
    author=AUTHOR,
    author_email=EMAIL,
    python_requires=REQUIRES_PYTHON,
    url=URL,
    packages=find_packages(exclude=('generators*',)),
    ext_modules=ext_modules,
    install_requires=REQUIRED,
    include_package_data=True,
    license='MIT',
    classifiers=[
        # Trove classifiers
        # Full list: https://pypi.python.org/pypi?%3Aaction=list_classifiers
        'License :: OSI Approved :: MIT License',
        'Operating System :: Microsoft :: Windows',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: Implementation :: CPython',
    ],
    cmdclass={
        'build_ext': BuildExt,
    },
)

print("build time: %.2f s" % (time() - t,))
