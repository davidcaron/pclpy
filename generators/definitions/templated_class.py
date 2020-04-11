import re
from typing import List

from CppHeaderParser import CppClass
from inflection import camelize

from generators.config import INDENT, DONT_HOLD_WITH_BOOST_SHARED_PTR, EXTRA_FUNCTIONS, GLOBAL_PCL_IMPORTS
from generators.definitions.constructor import Constructor
from generators.definitions.enum import Enum
from generators.definitions.method import Method
from generators.definitions.method import filter_template_types
from generators.definitions.variable import Variable
from generators.point_types_utils import clean_inheritance
from generators.utils import clean_doxygen


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
                py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
                cls.def(py::init<>());
                cls.def_property("stuff", &Class::getStuff, &Class::setStuff);
                cls.def("go", &Class::go);
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
        self.template = self.parse_template()
        self.is_templated = False

    def parse_template(self):
        template = self.class_.get("template")
        if template:
            for global_import in GLOBAL_PCL_IMPORTS:
                # add pcl:: to explicit global pcl imports
                with_pcl = "pcl::%s" % global_import
                if re.search(r"[ =]%s\W" % global_import, template):
                    template = template.replace(global_import, with_pcl)
        return template

    def to_str(self):
        if self.is_templated:
            s = 'py::class_<Class{inherits}{ptr}> {cls_var}(m, suffix.c_str(), R"({doc})")'
        else:
            s = 'py::class_<Class{inherits}{ptr}> {cls_var}(m, "{name}", R"({doc})")'
        ptr = ", boost::shared_ptr<Class>"
        if self.class_["name"] in DONT_HOLD_WITH_BOOST_SHARED_PTR:
            ptr = ""
        data = {
            "name": self.class_name,
            "cls_var": self.CLS_VAR,
            # "original_name": self.class_name,
            "inherits": (", %s" % self.inherits) if self.inherits else "",
            "ptr": ptr,
            "doc": clean_doxygen(self.class_.get("doxygen", "")),
        }
        return s.format(**data)

    def get_namespace(self):
        namespace = self.class_["namespace"]
        if namespace:
            namespace += "::"
        return namespace

    def typedefs(self):
        return self.class_["typedefs"]["public"]

    def to_class_function_definition(self, ind="") -> str:
        """
        template <typename PointInT, typename PointOutT>
        void define...(py::module &m, std::string const & suffix) { ... }
        """
        i = INDENT
        s = []
        if self.template:
            template_info = re.findall(r"<(.+)>", str(self.template.replace("\n", "")))

            types = ", ".join(filter_template_types(template_info[0]))
            # <PointXYZT, PointRGBT=PointXYZT> -> <PointXYZT, PointRGBT>
            types = re.sub(r"=[:\w]+", "", types)
            if types:
                self.is_templated = True
                s = []
                a = s.append
                a("{ind}{template}")
                a("{ind}void define{sub}{name}(py::module &m, std::string const & suffix) {ob}")
                templated_name = "{name}<%s>;" % types
                a("{ind}{i}using Class = typename {namespace}%s" % templated_name)
                for typedef in self.typedefs():
                    a("{ind}{i}using {typedef} = typename Class::{typedef};".format(ind=ind, i=i, typedef=typedef))
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
        if self.class_name in EXTRA_FUNCTIONS:
            s.append(EXTRA_FUNCTIONS[self.class_name])
        data = {
            "ind": ind,
            "i": i,
            "ob": "{ob}",  # will get formatted later
            "cb": "{cb}",
        }
        return "\n".join([line.format(**data) for line in s])
