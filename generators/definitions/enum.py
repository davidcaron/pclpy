from CppHeaderParser import CppVariable

from generators.config import INDENT


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

    def to_str(self, prefix, class_var_name):
        prefix = prefix[:-2] if prefix.endswith("::") else prefix
        s = []
        a = s.append
        a('using {cppname_lower} = {typename}{class_name}{cppname};')
        a('{i}py::enum_<{cppname_lower}>({parent_var}, "{name}")')
        for value in self.cppenum["values"]:
            a('{i}{i}.value("%s", {class_name}{cppname}::%s)' % (value["name"], value["name"]))
        a("{i}{i}.export_values()")
        typename = "typename " if "<" in self.cppenum["name"] or prefix == "Class" else ""
        data = {"name": self.name,
                "i": INDENT,
                "cppname": self.cppenum["name"],
                "cppname_lower": self.cppenum["name"].lower(),
                "class_name": ("%s::" % prefix) if prefix else "",
                "parent_var": class_var_name,
                "typename": typename
                }
        ret_val = "\n".join(s).format(**data)
        return ret_val

    def __repr__(self):
        return "<Enum %s>" % (self.name,)
