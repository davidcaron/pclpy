from generators.config import INDENT

from CppHeaderParser import CppVariable


class Enum:
    def __init__(self, enum: CppVariable):
        """
        Generates definition for an enum
        Example:
            py::enum_<Pet::Kind>(pet, "Kind")
                .value("Dog", Pet::Kind::Dog)
                .value("Cat", Pet::Kind::Cat)
                .export_values();
        """
        self.cppenum = enum
        self.name = enum["name"]

    def to_str(self, class_name, class_var_name):
        s = []
        a = s.append
        a('py::enum_<Class::{cppname}>({parent_var}, "{name}")')
        for value in self.cppenum["values"]:
            a('{i}{i}.value("%s", {class_name}::{cppname}::%s)' % (value["name"], value["name"]))
        a("{i}{i}.export_values()")
        data = {"name": self.name,
                "i": INDENT,
                "cppname": self.cppenum["name"],
                "class_name": class_name,
                "parent_var": class_var_name,
                }
        ret_val = "\n".join(s).format(**data)
        return ret_val

    def __repr__(self):
        return "<Enum %s>" % (self.name,)
