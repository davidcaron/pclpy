from CppHeaderParser import CppVariable


class Variable:
    def __init__(self, variable: CppVariable):
        """
        Generates definition for a variable
        Example:
            cls.def("indices", &Class::indices);
        """
        self.cppvariable = variable
        self.name = variable["name"]
        self.is_an_override = False

    def to_str(self, class_name, class_var_name):
        s = '{cls_var}.def_read{only}{static}("{name}", &{cls}::{cppname})'
        readonly = self.cppvariable["constant"] or self.cppvariable.get("array")
        data = {"name": self.name,
                "cls": class_name,
                "cls_var": class_var_name,
                "cppname": self.cppvariable["name"],
                "only": "only" if readonly else "write",
                "static": "_static" if self.cppvariable["static"] else "",
                }
        ret_val = s.format(**data)
        return ret_val

    def __repr__(self):
        return "<Variable %s>" % (self.name,)
