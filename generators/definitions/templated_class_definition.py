import re
from typing import List

from CppHeaderParser import CppClass
from inflection import camelize

from generators.definitions.constructor import Constructor
from generators.definitions.enum import Enum
from generators.definitions.method import Method
from generators.definitions.variable import Variable
from generators.config import INDENT
from generators.definitions.method import filter_template_types
from generators.point_types_utils import clean_inheritance


class ClassDefinition:
    CLS_VAR = "cls"

    def __init__(self,
                 class_: CppClass,
                 constructors: List[Constructor],
                 variables: List[Variable],
                 other_methods: List[Method],
                 sub_module: str):
        """
        Generates a templated function to define a pybind11 py::class_ with its methods and properties

        Example:
            template <typename PointT>
            void defineSuperFunction(py::module &m, std::string const & suffix) {
                using Class = SuperFunction<PointT>;
                py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>>(m, suffix.c_str())
                    .def(py::init<>())
                    .def_property("stuff", &Class::getStuff, &Class::setStuff)
                    .def("go", &Class::go)
                    ;
            }
        """
        self.sub_module = sub_module
        self.class_ = class_
        self.inherits = None
        if class_["inherits"]:
            inherits_list = clean_inheritance(self.class_,
                                              replace_with_templated_typename=False,
                                              formatted=True)
            self.inherits = ", ".join(inherits_list)
        self.class_name = class_["name"]
        self.constructors = constructors
        self.variables = variables
        named_enums = [e for e in class_["enums"]["public"] if e.get("name")]
        self.enums = list(map(Enum, named_enums))
        self.other_methods = other_methods
        self.template = class_.get("template")
        self.is_templated = False

    def to_str(self):
        if self.is_templated:
            s = 'py::class_<Class{inherits}, {ptr}> {cls_var}(m, suffix.c_str())'
        else:
            s = 'py::class_<Class{inherits}, {ptr}> {cls_var}(m, "{name}")'
        data = {
            "name": self.class_name,
            "cls_var": self.CLS_VAR,
            # "original_name": self.class_name,
            "inherits": (", %s" % self.inherits) if self.inherits else "",
            "ptr": "boost::shared_ptr<Class>"
        }
        return s.format(**data)

    def get_namespace(self):
        namespace = self.class_["namespace"]
        if namespace:
            namespace += "::"
        return namespace

    def typedefs(self):
        return self.class_["typedefs"]["public"]

    def to_class_function_definition(self, ind=""):
        """
        template <typename PointInT, typename PointOutT>
        void define...(py::module &m, std::string const & suffix) { ... }
        """
        i = INDENT
        s = []
        if self.template:
            template_info = re.findall(r"<(.+)>", str(self.template.replace("\n", "")))

            types = ", ".join(filter_template_types(template_info[0]))
            if types:
                self.is_templated = True
                s = ["{ind}{template}"]
                a = s.append
                a("{ind}void define{sub}{name}(py::module &m, std::string const & suffix) {ob}")
                templated_name = "{name}<%s>;" % types
                a("{ind}{i}using Class = {namespace}%s" % templated_name)
                for typedef in self.typedefs():
                    a("{ind}{i}using {typedef} = Class::{typedef};".format(ind=ind, i=i, typedef=typedef))
                a(self.py_class_definition(ind=ind + i) + "\n{ind}{i}{i}")
                a("{cb}")
        if not self.is_templated:
            a = s.append
            a("{ind}void define{sub}{name}(py::module &m) {ob}")
            a("{ind}{i}using Class = {namespace}{name}{empty_template};")
            for typedef in self.typedefs():
                a("{ind}{i}using %s = Class::%s;" % (typedef, typedef))
            a(self.py_class_definition(ind=ind + i))
            a("{cb}")
        data = {
            "name": self.class_name,
            "ind": ind,
            "i": i,
            "namespace": self.get_namespace(),
            "sub": camelize(self.sub_module),
            "template": self.template,
            "empty_template": "<>" if not self.is_templated and self.template else "",
            "template_types": self.template,
            "ob": "{",
            "cb": "}"
        }
        return "\n".join([line.format(**data) for line in s])

    def py_class_definition(self, ind=""):
        i = INDENT
        class_enums_names = [v["name"] for e in self.enums for v in e.cppenum["values"]]
        s = ["{ind}%s;" % self.to_str()]
        s += ["{ind}%s;" % enum.to_str("Class", class_var_name=self.CLS_VAR) for enum in self.enums]
        s += ["{ind}%s;" % c.to_str(class_var_name=self.CLS_VAR, class_enums_names=class_enums_names)
              for c in self.constructors]
        s += ["{ind}%s;" % v.to_str("Class", class_var_name=self.CLS_VAR) for v in self.variables]
        templated_methods = [m for m in self.other_methods if m.templated_types]
        s += ["{ind}%s;" % m.to_str("Class", class_var_name=self.CLS_VAR) for m in self.other_methods
              if not m in templated_methods]
        s += ["{ind}%s;" % m for method in templated_methods for m in
              method.to_str("Class", class_var_name=self.CLS_VAR)]
        data = {
            "ind": ind,
            "i": i
        }
        return "\n".join([line.format(**data) for line in s])
