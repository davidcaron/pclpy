import os
import sys
from collections import OrderedDict
from os.path import join
from typing import List
from itertools import chain

from inflection import camelize

from CppHeaderParser import CppClass, CppMethod, CppVariable

from generators.definitions.method import Method
from generators.definitions.property import Property
from generators.definitions.variable import Variable
from generators.definitions.constructor import Constructor
from generators.constants import PCL_BASE, common_includes, INDENT


def make_methods_split_overloads(methods, needs_overloading: List[str] = None):
    if needs_overloading is None:
        needs_overloading = []
    overloads, unique = [], []
    for n, m1 in enumerate(methods):
        other_methods = methods[:n] + methods[n + 1:]
        if any(m1["name"] == m2["name"] for m2 in other_methods) or m1["name"] in needs_overloading:
            method = Method(m1)
            method.is_an_overload = True
            overloads.append(method)
        else:
            unique.append(Method(m1))
    return overloads, unique


def make_properties_split_overloads(setters, getters):
    setters_overloads, setters_unique = make_methods_split_overloads(setters)
    getters_overloads, getters_unique = make_methods_split_overloads(getters)

    props = [Property(s, getters_unique) for s in setters_unique]
    return setters_overloads + getters_overloads, props


def filter_templated_methods(methods: List[Method]):
    filtered = []
    for method in methods:
        if method.method.get("template"):
            message = "Warning: Templated method not implemented (%s)"
            print(message % (method.method["name"],))
        else:
            filtered.append(method)
    return filtered


def split_methods_by_type(methods: List[CppMethod],
                          class_variables: List[CppVariable],
                          needs_overloading: List[str]):
    methods_with_name = [m for m in methods if m["name"]]  # CppHeaderParser bug
    constructors_methods = [m for m in methods if m["constructor"] and not is_copy_constructor(m)]
    copy_const = [m for m in methods if m["constructor"] and is_copy_constructor(m)]
    destructors = [m for m in methods if m["destructor"]]
    setters = [m for m in methods if m["name"].startswith("set")]
    getters = [m for m in methods if m["name"].startswith("get")]

    identified_methods = chain(constructors_methods, copy_const, destructors, setters, getters)
    identified_methods_line_numbers = set([m["line_number"] for m in identified_methods])
    others_methods = [m for m in methods if m["line_number"] not in identified_methods_line_numbers]

    others_overloads, others_unique = make_methods_split_overloads(others_methods, needs_overloading)

    properties_overloads, properties = make_properties_split_overloads(setters, getters)
    variables = list(map(Variable, class_variables))
    constructors = list(map(Constructor, constructors_methods))

    others = others_overloads + others_unique + properties_overloads

    others = filter_templated_methods(others)

    return constructors, properties, variables, others


def filter_template_types(template_string):
    if not template_string:
        return []
    types = template_string.split(", ")
    keep = ["typename", "class", "unsigned"]
    types = [s.strip().split(" ")[1] for s in types if any(k in s for k in keep)]  # and "=" not in s]
    return types


def make_header_include_name(module, header_name, path_only=False):
    name = "/".join([module, header_name]) if module else header_name
    if path_only:
        return "pcl/%s" % name
    else:
        return "#include <pcl/%s>" % name


def explicit_includes(module, header_name):
    if (module, header_name) == ("geometry", "mesh_io.h"):
        return "#include <pcl/geometry/polygon_mesh.h>\n" \
               "#include <pcl/geometry/triangle_mesh.h>"
    elif(module, header_name) == ("segmentation", "plane_refinement_comparator.h"):
        return "#include <pcl/ModelCoefficients.h>"
    elif(module, header_name) == ("features", "narf_descriptor.h"):
        return "#include <pcl/range_image/range_image.h>"
    elif(module, header_name) == ("features", "from_meshes.h"):
        return "#include <pcl/Vertices.h>"
    elif(module, header_name) == ("common", "synchronizer.h"):
        return '#include <boost/thread/mutex.hpp>'
    return ""


def is_copy_constructor(method):
    name = method["name"]
    params = method["parameters"]
    return len(params) == 1 and name in params[0]["type"]


# def fix_pcl_exports(class_: CppClass):
#     """
#     Fix for when a class' name is parsed as "PCL_EXPORTS"
#     """
#     if class_["name"] == "PCL_EXPORTS":
#         const = [m for m in class_["methods"]["public"] if m["constructor"]]
#         if not const:
#             raise NotImplementedError
#         class_["name"] = const[0]["name"]
#     return class_


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

    headers_dependencies = {header: get_include_lines(join(PCL_BASE, *header), header[0]) for header in headers}

    headers_include_names = OrderedDict()  # output is sorted in the same way always
    for h in headers:
        headers_include_names[h] = make_header_include_name(*h, path_only=True)

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
