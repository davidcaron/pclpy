from CppHeaderParser import CppMethod

from generators.config import INHERITED_ENUMS, CUSTOM_OVERLOAD_TYPES
from generators.utils import make_namespace_class


class Constructor:
    def __init__(self, const: CppMethod):
        """
        Generates definition for a class constructor
        Example:
            .def(py::init<str>(), "info"_a="None")
        """
        self.cppconst = const
        self.params = self.cppconst["parameters"]
        self.filter_parameters()

    def filter_parameters(self):
        filtered = []
        for param in self.params:
            if param["type"] == "void":
                continue
            filtered.append(param)
        self.params = filtered

    def to_str(self, class_var_name, class_enums_names):

        def default(p):
            val = p.get("defaultValue", "")
            if val:
                if val in class_enums_names:
                    val = "Class::" + val
                val = val.replace(" ", "")  # fix for exponent and float values parsed with added spaces
                val = "=" + val
            return val

        def init_param_type(param):
            type_ = param["raw_type"]
            type_only_last_element = type_.split("::")[-1]
            class_with_param_name = (param["method"]["name"], param["raw_type"])
            class_typedefs = param["method"]["parent"]["typedefs"]["public"]
            custom = CUSTOM_OVERLOAD_TYPES.get((param["method"]["parent"]["name"], type_))
            if custom:
                type_ = custom
            elif param.get("enum"):
                type_ = "Class::%s" % param.get("enum").split("::")[-1]
            elif type_only_last_element in class_typedefs:
                type_ = type_only_last_element
            elif class_with_param_name in INHERITED_ENUMS:
                type_ = "Class::" + type_only_last_element
            if param.get("pointer"):
                type_ += "*"
            return type_

        if len(self.params):
            s = '{cls_var}.def(py::init<{params_types}>(), {params_names})'
            types = ", ".join([init_param_type(p) for p in self.params])
            names = ", ".join(['"%s"_a%s' % (p["name"], default(p)) for p in self.params])
            data = {"params_types": types,
                    "params_names": names,
                    "cls_var": class_var_name}
        else:
            s = '{cls_var}.def(py::init<>())'.format(cls_var=class_var_name)
            data = {}
        return s.format(**data)
