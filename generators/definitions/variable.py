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
        if self.cppvariable.get("array"):
            s = ('{cls_var}.def("{name}", [](py::object &obj) {ob} '
                 '{cls} &a = obj.cast<{cls}&>(); '
                 'return py::array_t<{type_}>({ob}{size}{cb}, {ob}sizeof({type_}) * {size}{cb}, a.{name}, obj); '
                 '{cb})')
        else:
            s = '{cls_var}.def_read{only}{static}("{name}", &{cls}::{name})'
        readonly = self.cppvariable["constant"]
        data = {"name": self.name,
                "cls": class_name,
                "cls_var": class_var_name,
                "size": self.cppvariable.get("array_size", ""),
                "type_": self.cppvariable["type"],
                "only": "only" if readonly else "write",
                "static": "_static" if self.cppvariable["static"] else "",
                "ob": "{ob}",  # will be formatted later
                "cb": "{cb}",
                }
        ret_val = s.format(**data)
        return ret_val

    def __repr__(self):
        return "<Variable %s>" % (self.name,)
