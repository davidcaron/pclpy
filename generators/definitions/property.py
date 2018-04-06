from typing import List

from inflection import underscore

from generators.definitions.method import Method


class Property:
    def __init__(self, setter_method: Method, getters_methods: List[Method]):
        """
        Generates definition for a property
        Example:
            .def_property("stuff", &Class::getStuff, &Class::setStuff)
        """
        self.setter = setter_method.method
        getters = [g.method for g in getters_methods]
        getter = [g for g in getters if g["name"][3:] == self.setter["name"][3:]]
        self.getter = getter[0] if getter else None
        self.name = underscore(self.setter["name"].replace("set", ""))

    def __eq__(self, other):
        return self.name == other.name

    def to_str(self, class_name, class_var_name, ind=""):
        if self.getter:
            s = '{cls_var}.def_property("{name}", &{cls}::{get}, &{cls}::{set})'
        else:
            s = '{cls_var}.def("set_{name}", &{cls}::{set})'
        data = {"name": self.name,
                "cls": class_name,
                "cls_var": class_var_name,
                "get": self.getter["name"] if self.getter else None,
                "set": self.setter["name"],
                }
        return s.format(**data)

    def __repr__(self):
        return "<Property %s>" % (self.name, )