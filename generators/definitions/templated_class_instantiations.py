from typing import List, Dict
import re

from inflection import camelize

from generators.config import INDENT
from generators.utils import function_definition_name
from generators.definitions.method import filter_template_types

from CppHeaderParser import CppClass


class TemplatedClassInstantiations:
    BASE_SUB_MODULE_NAME = "sub_module"

    def __init__(self,
                 sorted_classes: List[CppClass],
                 module: str,
                 header_name: str,
                 classes_point_types: Dict,
                 other_types: Dict,
                 ):
        """♠
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
        self.other_types = other_types

    def repr_sub_module(self, class_name: str):
        """
        Create a submodule for each templated class
        """
        s = 'py::module {sub} = {base}.def_submodule("{name}", "Submodule {name}")'
        data = {
            "sub": self.sub_module_name(class_name),
            "base": self.BASE_SUB_MODULE_NAME,
            "name": class_name
        }
        return s.format(**data)

    def to_module_function_definition(self, global_indent=""):
        """
        void define...Classes(py::module &m) { ... }
        """
        s = []
        a = s.append
        i = INDENT
        a("{ind}void define{sub}{name}Classes(py::module &{base}) {ob}")
        for line in self.generate_templated_function_calls():
            a("{ind}{i}%s;" % line)
        a("{ind}{cb}")

        data = {
            "ind": global_indent,
            "i": i,
            "base": self.BASE_SUB_MODULE_NAME,
            "name": function_definition_name(self.header_name),
            "sub": camelize(self.module),
            "ob": "{",
            "cb": "}"
        }
        return "\n".join([line.format(**data) for line in s])

    def sub_module_name(self, class_name):
        return "%s_%s" % (self.BASE_SUB_MODULE_NAME, class_name)

    def generate_templated_function_calls(self):
        s = []
        for c in self.sorted_classes:
            template_types = []
            if c.get("template"):
                template_info = re.findall(r"<(.+)>", str(c["template"].replace("\n", "")))
                if template_info:
                    template_types = filter_template_types(template_info[0])

            if template_types:
                sub_module_name = self.sub_module_name(c["name"])
                types_list = self.other_types.get(c["name"], self.classes_point_types.get(c["name"], []))
                if types_list:
                    add_pcl_prefix = c["name"] not in self.other_types
                    s.append(self.repr_sub_module(c["name"]))
                    for types in types_list:
                        define = 'define{sub}{name}<{types}>({sub_module_name}, "{types_names}")'
                        if add_pcl_prefix:
                            types_str = ", ".join(["pcl::" + t for t in types])
                        else:
                            types_str = ", ".join(types)
                        define = define.format(sub=camelize(self.module),
                                               name=c["name"],
                                               sub_module_name=sub_module_name,
                                               types=types_str,
                                               types_names="_".join(types))
                        s.append(define)
            elif c.get("template"):
                print("Warning: skipping definition for specialized templated class (%s %s)" %
                      (self.module, c["name"]))
            else:
                define = 'define{sub}{name}({sub_name})'
                define = define.format(sub=camelize(self.module), name=c["name"], sub_name=self.BASE_SUB_MODULE_NAME)
                s.append(define)
        return s
