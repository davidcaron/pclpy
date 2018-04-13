import sys
from collections import OrderedDict
from os.path import join
from typing import List

from inflection import camelize

from generators.constants import PCL_BASE, common_includes, INDENT, TEMPLATED_METHOD_TYPES


def make_header_include_name(module, header_name, path=None, path_only=False):
    name = path.replace("\\", "/") if path else "/".join([module, header_name]) if module else header_name
    if path_only:
        return "pcl/%s" % name
    else:
        return "#include <pcl/%s>" % name


def explicit_includes(module, header_name):
    if (module, header_name) == ("geometry", "mesh_io.h"):
        return "#include <pcl/geometry/polygon_mesh.h>\n" \
               "#include <pcl/geometry/triangle_mesh.h>"
    elif (module, header_name) == ("segmentation", "plane_refinement_comparator.h"):
        return "#include <pcl/ModelCoefficients.h>"
    elif (module, header_name) == ("features", "narf_descriptor.h"):
        return "#include <pcl/range_image/range_image.h>"
    elif (module, header_name) == ("features", "from_meshes.h"):
        return "#include <pcl/Vertices.h>"
    elif (module, header_name) == ("common", "synchronizer.h"):
        return '#include <boost/thread/mutex.hpp>'
    elif (module, header_name) == ("visualization", "pcl_visualizer.h"):
        return "#pragma warning(disable : 4996)"
    elif (module, header_name) == ("recognition", "orr_octree.h"):
        return "#pragma warning(disable : 4800)"
    elif (module, header_name) == ("recognition", "obj_rec_ransac.h"):
        return "#pragma warning(disable : 4267)"
    elif (module, header_name) == ("recognition", "model_library.h"):
        return "#pragma warning(disable : 4267)"
    elif (module, header_name) == ("surface", "concave_hull.h"):
        return "#pragma warning(disable : 4996)"
    return ""


def make_namespace_class(namespace, class_name):
    while namespace.endswith(":"):
        namespace = namespace[:-1]
    class_name = class_name.strip()
    template_info = ""
    if "<" in class_name:
        p1, p2 = class_name.find("<"), class_name.rfind(">")
        template_string = class_name[p1 + 1:p2]
        other_stuff = class_name[p2 + 1:]
        class_name = class_name[:p1]
        template_info = ", ".join([make_namespace_class(namespace, t.strip()) for t in template_string.split(",")])
        template_info = "<%s>%s" % (template_info, other_stuff)
    if not namespace.startswith("pcl"):
        namespace = "pcl::%s" % namespace

    if class_name.startswith("pcl"):
        pass
    elif class_name in TEMPLATED_METHOD_TYPES:
        pass
    else:
        merged = "%s::%s" % (namespace, class_name)
        nonrepeating = []
        for name in merged.split("::"):
            if not nonrepeating or name != nonrepeating[-1]:
                nonrepeating.append(name)
        class_name = "::".join(nonrepeating)
    return class_name + template_info


def function_definition_name(header_name):
    return camelize(header_name.replace(".h", "")).replace(" ", "")


def sort_headers_by_dependencies(headers):
    headers = list(sorted(headers))

    def get_include_lines(path, module):
        try:
            lines = open(path).readlines()
        except UnicodeDecodeError:
            lines = open(path, encoding="utf8").readlines()
        headers = []
        for line in lines:
            stripped = line.strip()
            if stripped.startswith("#include"):
                include_string = stripped[10:-1]
                headers.append(include_string)
                # fix for relative_imports
                headers.append(make_header_include_name(module, include_string, path_only=True))
        return headers

    make_header_path = lambda module, header, path: join(PCL_BASE, path) if path else join(PCL_BASE, module, header)
    headers_dependencies = {header: get_include_lines(make_header_path(*header), header[0]) for header in headers}

    headers_include_names = OrderedDict()  # output is sorted in the same way always
    for h in headers:
        headers_include_names[h] = make_header_include_name(h[0], h[1], path=h[2], path_only=True)

    sorted_headers = []
    while headers_include_names:
        for header in headers_include_names.keys():
            dependencies = headers_dependencies[header]
            if not any(h in dependencies for h in headers_include_names.values()):
                sorted_headers.append(header)
                del headers_include_names[header]
                break
        else:
            if all(h[0] == "outofcore" for h in headers_include_names.keys()):
                # special case for outofcore which seems to contain circular dependencies
                for header in sorted(headers_include_names.keys(), key=lambda x: len(x[1])):
                    sorted_headers.append(header)
                    del headers_include_names[header]
            else:
                print("Error: circular dependencies?")
                for h in headers_include_names:
                    print(h)
                sys.exit(1)
    return sorted_headers


def generate_main_loader(modules):
    modules = list(sorted(modules))
    s = [common_includes]
    a = s.append
    for module in modules:
        a("void define%sClasses(py::module &);" % camelize(module))
    a("")
    a("void defineClasses(py::module &m) {")
    for module in modules:
        a("%sdefine%sClasses(m);" % (INDENT, camelize(module)))
    a("}")
    return "\n".join(s)


def parentheses_are_balanced(line, parenthesis):
    stack = []
    opened, closed = parenthesis
    for c in line:
        if c == opened:
            stack.append(c)
        elif c == closed:
            if not stack.pop() == opened:
                return False
    return not stack


def split_overloads(methods, needs_overloading: List[str] = None):
    if needs_overloading is None:
        needs_overloading = []
    overloads, unique = [], []
    for n, m1 in enumerate(methods):
        other_methods = methods[:n] + methods[n + 1:]
        if any(m1["name"] == m2["name"] for m2 in other_methods) or m1["name"] in needs_overloading:
            overloads.append(m1)
        else:
            unique.append(m1)
    return overloads, unique
