import re
from collections import OrderedDict
from typing import List, Dict

from CppHeaderParser import CppClass, CppVariable
from inflection import camelize

from generators.config import INDENT, KEEP_ASIS_TYPES, BASE_SUB_MODULE_NAME
from generators.definitions.enum import Enum
from generators.definitions.method import filter_template_types
from generators.definitions.module_variable import define_variable
from generators.utils import function_definition_name


class Instantiations:
    def __init__(self,
                 sorted_classes: List[CppClass],
                 module: str,
                 header_name: str,
                 classes_point_types: OrderedDict,
                 variables: List[CppVariable],
                 enums: List[CppVariable],
                 ):
        """
        Generate templated function calls that instantiate pybind11 classes
        Example:
            void defineSuperFunctionClasses(py::module &sub_module) {
                defineSuperFunction<PointXYZ>(sub_module, "PointXYZ");
                defineSuperFunction<PointXYZI>(sub_module, "PointXYZI");
                defineSuperFunction<PointXYZRGBA>(sub_module, "PointXYZRGBA");
                defineSuperFunction<PointXYZRGB>(sub_module, "PointXYZRGB");
            }
        """
        self.sorted_classes = sorted_classes
        self.module = module
        self.header_name = header_name
        self.classes_point_types = classes_point_types
        self.variables = variables
        self.enums = enums

    def repr_sub_module(self, class_name: str):
        """
        Create a submodule for each templated class
        """
        s = 'py::module {sub} = {base}.def_submodule("{name}", "Submodule {name}")'
        data = {
            "sub": self.sub_module_name(class_name),
            "base": BASE_SUB_MODULE_NAME,
            "name": class_name
        }
        return s.format(**data)

    def generate_instantiation_function(self, global_indent="", has_functions=False):
        """
        void define...Classes(py::module &m) { ... }
        """
        s = []
        a = s.append
        i = INDENT
        a("{ind}void define{sub}{name}Classes(py::module &{base}) {ob}")
        for var in self.variables:
            a("{ind}{i}%s" % define_variable(var))
        for enum in self.enums:
            enum = Enum(enum)
            a("{ind}{i}%s;" % enum.to_str(prefix=enum.cppenum["namespace"], class_var_name=BASE_SUB_MODULE_NAME))
        for line in self.generate_templated_class_calls():
            a("{ind}{i}%s;" % line)
        if has_functions:
            a("{ind}{i}define{sub}{name}Functions({base});")
        a("{ind}{cb}")

        data = {
            "ind": global_indent,
            "i": i,
            "base": BASE_SUB_MODULE_NAME,
            "name": function_definition_name(self.header_name),
            "sub": camelize(self.module),
            "ob": "{",
            "cb": "}"
        }
        return "\n".join([line.format(**data) for line in s])

    def sub_module_name(self, class_name):
        return "%s_%s" % (BASE_SUB_MODULE_NAME, class_name)

    def generate_templated_class_calls(self):
        s = []
        for c in self.sorted_classes:
            template_types = []
            if c.get("template"):
                template_info = re.findall(r"<(.+)>", str(c["template"].replace("\n", "")))
                if template_info:
                    template_types = filter_template_types(template_info[0])

            if template_types:
                sub_module_name = self.sub_module_name(c["name"])
                types_list = self.classes_point_types.get(c["name"], [])
                if types_list:
                    s.append(self.repr_sub_module(c["name"]))
                    for types in types_list:
                        type_names = "_".join(map(format_class_name, types))
                        define = 'define{sub}{name}<{types}>({sub_module_name}, "{types_names}")'
                        add_pcl = lambda t: ("pcl::" + t) if t not in KEEP_ASIS_TYPES else t
                        types_str = ", ".join([add_pcl(t) for t in types])
                        define = define.format(sub=camelize(self.module),
                                               name=c["name"],
                                               sub_module_name=sub_module_name,
                                               types=types_str,
                                               types_names=type_names)
                        s.append(define)
            else:
                define = 'define{sub}{name}({sub_name})'
                define = define.format(sub=camelize(self.module), name=c["name"], sub_name=BASE_SUB_MODULE_NAME)
                s.append(define)
        return s


def format_class_name(value):
    """
    Example:
        octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ> -> OctreePointCloudVoxelCentroidContainer_PointXYZ
    """
    if "<" in value:
        before = format_class_name(value[:value.find("<")])
        inside = format_class_name(value[value.find("<") + 1:value.rfind(">")])
        after = format_class_name(value[value.rfind(">")+1:])
        return "_".join([v for v in (before, inside, after) if v])
    elif "::" in value:
        return value[value.rfind("::") + 2:]
    else:
        return value
