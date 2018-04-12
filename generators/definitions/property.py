from typing import List

# from generators.definitions.method import Method
from CppHeaderParser import CppMethod
from generators.utils import split_overloads
from generators.definitions.method_parameters import make_pybind_argument_list


class Property:
    def __init__(self, setter_method: CppMethod, getters_methods: List[CppMethod]):
        """
        Generates definition for a property
        Example:
            .def_property("stuff", &Class::getStuff, &Class::setStuff)
        """
        self.cppsetter = setter_method
        getter = [g for g in getters_methods if g["name"][3:] == self.cppsetter["name"][3:]]
        self.getter = getter[0] if getter else None
        self.name = self.cppsetter["name"].replace("set", "")

    def __eq__(self, other):
        return self.name == other.name

    def to_str(self, class_name, class_var_name, ind=""):
        args = make_pybind_argument_list(self.cppsetter["parameters"])
        if self.getter:
            s = '{cls_var}.def_property("{name}", &{cls}::{get}, &{cls}::{set})'
        else:
            s = '{cls_var}.def("set_{name}", &{cls}::{set}{args})'
        data = {"name": self.name,
                "cls": class_name,
                "cls_var": class_var_name,
                "get": self.getter["name"] if self.getter else None,
                "set": self.cppsetter["name"],
                "args": args,
                }
        return s.format(**data)

    def __repr__(self):
        return "<Property %s>" % (self.name, )


def make_properties_split_overloads(setters, getters):
    setters_overloads, setters_unique = split_overloads(setters)
    getters_overloads, getters_unique = split_overloads(getters)

    props = [Property(s, getters_unique) for s in setters_unique]
    return setters_overloads + getters_overloads, props