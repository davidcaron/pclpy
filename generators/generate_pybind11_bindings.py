import os
import platform
import shutil
import sys
from collections import Counter
from collections import defaultdict, OrderedDict
from os.path import join
from typing import List, Dict, Set

from CppHeaderParser import CppHeaderParser
from CppHeaderParser.CppHeaderParser import CppMethod

import generators.dependency_tree
from generators.config import common_includes, PCL_BASE, PATH_LOADER, PATH_MODULES, MODULES_TO_BUILD, \
    HEADERS_TO_SKIP, ATTRIBUTES_TO_SKIP, CLASSES_TO_IGNORE, METHODS_TO_SKIP, SUBMODULES_TO_SKIP, EXPLICIT_INCLUDES, \
    SPECIALIZED_TEMPLATED_TYPES_TO_SKIP
from generators.definitions.function import generate_function_definitions, get_methods_defined_outside
from generators.definitions.method import split_methods_by_type
from generators.definitions.submodule_loader import generate_loader
from generators.definitions.templated_class import ClassDefinition
from generators.instantiations import Instantiations
from generators.point_types_utils import unpack_yaml_point_types
from generators.utils import make_header_include_name, sort_headers_by_dependencies, \
    generate_main_loader, make_namespace_class, read_header_file


def filter_methods_for_parser_errors(methods):
    return [m for m in methods if not m["name"] in ("void", "bool")]


def filter_methods_to_skip(methods):
    filtered_methods = []
    for m in methods:
        if (m["parent"]["name"], m["name"]) in METHODS_TO_SKIP:
            continue
        if "Callback" in m["name"]:
            single_argument = len(m["parameters"]) == 1
            boost_function = single_argument and m["parameters"][0]["type"].startswith("boost::function")
            if not boost_function:
                continue
        filtered_methods.append(m)
    return filtered_methods


def same_parameters(p1: Dict, p2: Dict) -> bool:
    fields = ["constant", "name", "raw_type", "reference", "static"]
    return all(p1[f] == p2[f] for f in fields)


def same_methods(m1: CppMethod, m2: CppMethod) -> bool:
    if m1["name"] != m2["name"]:
        return False

    # bug in CppHeaderParser
    # in "void ImageGrabber<PointT>::publish", "void ImageGrabber<PointT>::" is the return type
    path = m1.get("path", m2.get("path"))
    path = path[path.rfind(":") + 1:]
    if not any(path in type_ for type_ in [m1["rtnType"], m2["rtnType"]]):
        return False

    # same parameters
    for p1 in m1["parameters"]:
        for p2 in m2["parameters"]:
            if m1["name"] == m2["name"] and same_parameters(p1, p2):
                break
        else:
            return False
    return len(m1["parameters"]) == len(m2["parameters"])


def private_methods_defined_outside(private_methods: List[CppMethod],
                                    methods_declared_outside: List[CppMethod]) -> List[CppMethod]:
    private_defined_outside = []
    for m_private in private_methods:
        for m_outside in methods_declared_outside:
            if same_methods(m_private, m_outside):
                private_defined_outside.append(m_private)
                break
    return private_defined_outside


def generate_class_definitions(main_classes,
                               module,
                               header_name,
                               path,
                               needs_overloading: List[str],
                               methods_defined_outside: List[CppMethod]) -> str:
    text = []
    a = text.append
    a(common_includes)
    a(EXPLICIT_INCLUDES.get((module, header_name), ""))
    a(make_header_include_name(module, header_name, path))
    a("")

    namespaces = set([c["namespace"] for c in main_classes])
    for namespace in namespaces:
        if not namespace == "pcl":
            a("using namespace %s;" % namespace)
    a("\n")

    for class_ in main_classes:
        methods = class_["methods"]["public"]
        methods = filter_methods_for_parser_errors(methods)
        methods = filter_methods_to_skip(methods)
        private_and_protected = class_["methods"]["private"] + class_["methods"]["protected"]
        methods += private_methods_defined_outside(private_and_protected, methods_defined_outside)
        class_properties = [p for p in class_["properties"]["public"]
                            if not "using" in p["type"]
                            and not "union" in p["type"]]
        union_properties = [p for nested_class in class_["nested_classes"]
                            for p in nested_class["properties"]["public"]
                            if "union" in nested_class["name"]]
        class_properties += union_properties
        class_properties = filter_class_properties(module, header_name, class_["name"], class_properties)
        constructors, variables, others = split_methods_by_type(methods, class_properties,
                                                                needs_overloading)
        if not class_["can_be_instantiated"]:
            constructors = []
        class_def = ClassDefinition(class_, constructors, variables, others, module)
        a(class_def.to_class_function_definition())
        a("")

    return "\n".join(text)


def filter_class_properties(module, header, class_name, properties):
    key = (module, header, class_name)
    # ignore properties without a name
    properties = [p for p in properties if p["name"]]
    if key in ATTRIBUTES_TO_SKIP:
        to_ignore = ATTRIBUTES_TO_SKIP[key]
        filtered_properties = []
        for p in properties:
            if p["name"] in to_ignore:
                continue
            filtered_properties.append(p)
        properties = filtered_properties
    return properties


def get_main_classes(header, module, header_name):
    # header = read_headers(base_path, header_name, module)
    main_classes = [c for c in header.classes.values() if c["namespace"] in ("pcl", "pcl::" + module)]
    filtered_main_classes = []
    for class_ in main_classes:
        specialized_template = class_.get("template") and "<" in class_["name"]
        if specialized_template:
            to_skip = any(("<%s>" % type_) in class_["name"] for type_ in SPECIALIZED_TEMPLATED_TYPES_TO_SKIP)
            if not to_skip:
                message = "Warning: Template class specialization not implemented for class %s in %s"
                print(message % (class_["name"], header_name))
        elif (module, header_name, class_["name"]) in CLASSES_TO_IGNORE:
            pass
        else:
            filtered_main_classes.append(class_)
    filtered_main_classes = sorted(filtered_main_classes, key=lambda c: c["name"])
    return filtered_main_classes


def get_functions(header, module):
    functions = [f for f in header.functions if f["namespace"] in ("pcl",
                                                                   "pcl::",
                                                                   "pcl::%s" % module,
                                                                   "pcl::%s::" % module)]
    functions = sorted(functions, key=lambda f: f["name"])
    filtered = filter_module_level_functions(functions)
    return filtered


def filter_module_level_functions(functions: List[CppMethod]):
    filtered = []
    for f in functions:
        keep = True
        if f.get("returns_const"):
            keep = False
        for param in f["parameters"]:
            for type_ in SPECIALIZED_TEMPLATED_TYPES_TO_SKIP:
                if type_ in param["type"]:
                    keep = False
        if keep:
            filtered.append(f)
    return filtered


def get_variables(header):
    variables = [v for v in header.variables if v.get("defaultValue")]
    variables = sorted(variables, key=lambda v: v["name"])
    return variables


def get_enums(header):
    enums = [e for e in header.enums if e.get("name")]  # skip nameless enums
    enums = sorted(enums, key=lambda v: v["name"])
    return enums


def read_header(header_path, skip_macros=None):
    # I tried to do this in multiple threads but it seems like CppHeaderParser is not thread safe...
    if skip_macros is None:
        skip_macros = []
    header_file_str = read_header_file(header_path, skip_macros)
    parser = CppHeaderParser
    parser.debug = False
    header = parser.CppHeader(header_file_str, argType="string")
    return header


def clean():
    try:
        os.remove(PATH_LOADER)
    except FileNotFoundError:
        pass
    if os.path.exists(PATH_MODULES):
        shutil.rmtree(PATH_MODULES)


def check_if_needs_overloading(main_classes):
    needs_overloading = {}
    classes_by_module = defaultdict(list)
    for (module, _), class_ in main_classes.items():
        classes_by_module[module] += class_

    for module, classes in classes_by_module.items():
        needs = []
        for class_ in classes:
            count = Counter(m["name"] for methods in class_["methods"].values() for m in methods)
            for name, count in count.items():
                if count >= 2:
                    needs.append(name)
        needs_overloading[module] = needs

    return needs_overloading


def get_headers(modules=None, skip_modules=None):
    def listmod(module):
        found_modules = []
        for base, folders, files in os.walk(join(PCL_BASE, module)):
            if any(base.endswith(m) for m in SUBMODULES_TO_SKIP):
                continue
            relative_base = os.path.abspath(base).replace(PCL_BASE, "")[1:]
            for f in files:
                if f.endswith(".h"):
                    found_modules.append([f, join(relative_base, f)])
        return found_modules

    if modules is None:
        modules = MODULES_TO_BUILD

    if skip_modules is not None:
        modules = [m for m in modules if m not in skip_modules]

    headers_to_generate = [(module, header_name, path) for module in modules
                           for header_name, path in listmod(module)]
    base_headers = [("", f, f) for f in os.listdir(PCL_BASE) if f.endswith(".h")]
    headers_to_generate += base_headers

    headers_to_generate_temp = []
    for module, header_name, path in headers_to_generate:
        if (module, header_name) in HEADERS_TO_SKIP:
            continue
        headers_to_generate_temp.append(tuple([module, header_name, path]))

    return headers_to_generate_temp


def get_pure_virtual_methods(class_: CppHeaderParser.CppClass) -> Set[str]:
    access = "private protected public".split()
    return set([m["name"] for a in access for m in class_["methods"][a] if m["pure_virtual"]])


def get_all_class_methods_not_pure_virtual(class_: CppHeaderParser.CppClass) -> Set[str]:
    access = "private protected public".split()
    return set([m["name"] for a in access for m in class_["methods"][a] if not m["pure_virtual"]])


def flag_instantiatable_class(dependency_tree, main_classes):
    """determine if the class can be instantiated"""
    main_classes_by_name_namespace = {make_namespace_class(c["namespace"], c["name"]): c
                                      for classes in main_classes.values() for c in classes}

    for module, header_name in main_classes:
        for class_ in main_classes[(module, header_name)]:
            can_be_instantiated = True
            if class_["abstract"]:
                can_be_instantiated = False
            else:
                # check if any pure virtual method is not implemented
                all_implemented_inherited_methods = get_all_class_methods_not_pure_virtual(class_)
                namespace_class = make_namespace_class(class_["namespace"], class_["name"])
                for base_name_nsp in dependency_tree.breadth_first_iterator(namespace_class):
                    base_class = main_classes_by_name_namespace.get(base_name_nsp)
                    if base_class:
                        base_class_methods = get_all_class_methods_not_pure_virtual(base_class)
                        all_implemented_inherited_methods.update(base_class_methods)

                for base_name_nsp in dependency_tree.breadth_first_iterator(namespace_class):
                    base_class = main_classes_by_name_namespace.get(base_name_nsp)
                    if base_class and base_class["abstract"]:
                        base_pure_virtual_methods = get_pure_virtual_methods(base_class)
                        if base_pure_virtual_methods - all_implemented_inherited_methods:
                            can_be_instantiated = False

            class_["can_be_instantiated"] = can_be_instantiated


def load_yaml_point_types(not_every_point_type):
    classes_point_types = unpack_yaml_point_types("point_types_generated.yml", not_every_point_type)
    extra_point_types = unpack_yaml_point_types("point_types_extra.yml")
    for k, v in extra_point_types.items():
        if k in classes_point_types:
            classes_point_types[k].append(v)
        else:
            classes_point_types[k] = v
    return classes_point_types


def make_module_dirs(modules):
    for module in modules:
        module_dir = join(PATH_MODULES, module)
        if not os.path.exists(module_dir):
            os.makedirs(module_dir)


def is_file_different(path, text):
    v = open(path).read()
    if v != text:
        print("File is different: %s" % os.path.split(path)[1])
        return True
    # print("File is the same: %s" % os.path.split(path)[1])
    return False


def write_if_different(files_to_write, delete_others):
    written = []

    for base, folder, files in os.walk(PATH_MODULES):
        for f in files:
            path = join(base, f)
            if path in files_to_write:
                if is_file_different(path, files_to_write[path]):
                    open(path, "w").write(files_to_write[path])
                    written.append(path)
            elif delete_others:
                os.remove(path)
                print("Deleted: " + path)

    # write new files
    for path, text in files_to_write.items():
        if path not in written:
            open(path, "w").write(files_to_write[path])


def delete_other_dirs(modules):
    for f in os.listdir(PATH_MODULES):
        folder = join(PATH_MODULES, f)
        if f not in modules and os.path.isdir(folder):
            shutil.rmtree(folder, ignore_errors=True)


def write_stuff_if_needed(generated_headers: OrderedDict, delete_others=True):
    modules = set(module for module, _ in generated_headers.keys())

    make_module_dirs(modules)

    # hpp
    files_to_write = {}
    for (module, header_name), text in generated_headers.items():
        if text:
            output_path = join(PATH_MODULES, module, header_name + "pp")
            files_to_write[output_path] = text

    # loaders
    loader_modules = defaultdict(list)
    for (module, header_name), text in generated_headers.items():
        if text:
            loader_modules[module or "base"].append(header_name)
    for module, headers in loader_modules.items():
        path_loader = join(PATH_MODULES, "_%s_loader.cpp" % module)
        files_to_write[path_loader] = generate_loader(module, headers)

    files_to_write[PATH_LOADER] = generate_main_loader(loader_modules)

    write_if_different(files_to_write, delete_others)

    if delete_others:
        delete_other_dirs(modules)


def generate(headers_to_generate, skip_macros, not_every_point_type=False) -> OrderedDict:
    """
    :return: OrderedDict
    """
    main_classes, module_functions, module_variables, module_enums = {}, {}, {}, {}

    for module, header_name, path in headers_to_generate[:]:
        header_full_path = join(PCL_BASE, path) if path else join(PCL_BASE, module, header_name)
        header = read_header(header_full_path, skip_macros)
        main_classes[(module, header_name)] = get_main_classes(header, module, header_name)
        module_functions[(module, header_name)] = get_functions(header, module)
        module_variables[(module, header_name)] = get_variables(header)
        module_enums[(module, header_name)] = get_enums(header)

    classes = [c for module, header, path in headers_to_generate
               for c in main_classes[(module, header)]]

    dependency_tree = generators.dependency_tree.DependencyTree(classes)

    loaded_point_types = load_yaml_point_types(not_every_point_type)
    classes_point_types: OrderedDict = dependency_tree.get_point_types_with_dependencies(loaded_point_types)

    classes_sorted_base_first = list(dependency_tree.leaf_iterator())

    def index_for_class(class_):
        return classes_sorted_base_first.index(make_namespace_class(class_["namespace"], class_["name"]))

    # sort classes inside modules based on inheritance
    for module, header in main_classes:
        main_classes[(module, header)] = list(sorted(main_classes[(module, header)], key=index_for_class))

    headers_to_generate = sort_headers_by_dependencies(headers_to_generate, skip_macros=skip_macros)

    methods_need_overloading = check_if_needs_overloading(main_classes)

    flag_instantiatable_class(dependency_tree, main_classes)

    def generate_header(module, header, path, keep_if_no_instantiation) -> str:
        header_functions = module_functions[(module, header)]
        header_classes = main_classes[(module, header)]

        methods_defined_outside = get_methods_defined_outside(header_functions)

        class_definitions = generate_class_definitions(header_classes,
                                                       module,
                                                       header,
                                                       path,
                                                       methods_need_overloading.get(module),
                                                       methods_defined_outside)

        function_definitions = generate_function_definitions(header_functions,
                                                             module,
                                                             header,
                                                             not_every_point_type=not_every_point_type)
        instantiations = Instantiations(header_classes,
                                        module,
                                        header,
                                        classes_point_types,
                                        module_variables[(module, header)],
                                        module_enums[(module, header)],
                                        )
        instantiation_function = instantiations.generate_instantiation_function(has_functions=bool(header_functions))
        something_instantiated = len(instantiation_function.split("\n")) > 2
        text = []
        if something_instantiated or keep_if_no_instantiation:
            text = [class_definitions, function_definitions, instantiation_function]

        return "\n".join(text)

    generated_headers = OrderedDict()
    for module, header, path in headers_to_generate:
        generated_headers[(module, header)] = generate_header(module, header, path, keep_if_no_instantiation=False)

    return generated_headers


def main():
    import time
    t = time.time()

    windows = platform.system() == "Windows"

    skip_macros = []
    skip_modules = []

    if not windows:
        skip_macros = ["_MSC_VER"]
        skip_modules = ["visualization"]

    all_headers = get_headers(skip_modules=skip_modules)
    not_every_point_type = "--not-every-point-type" in sys.argv
    generated_headers = generate(all_headers, skip_macros, not_every_point_type)
    write_stuff_if_needed(generated_headers, delete_others=True)

    print("generated in %.2f s" % (time.time() - t,))


if __name__ == '__main__':
    main()
