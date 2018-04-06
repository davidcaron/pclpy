from inflection import underscore

from CppHeaderParser import CppMethod

from generators.constants import CUSTOM_OVERLOAD_TYPES, EXPLICIT_IMPORTED_TYPES, KEEP_DISAMIGUATION_TYPES_STARTSWITH


class Method:
    def __init__(self, method: CppMethod):
        """
        Generates definition for a method
        Example:
            .def("go", &Class::go)
        """
        self.method = method
        self.name = underscore(method["name"])
        self.is_an_overload = False

    def make_disambiguation(self, class_name):
        if len(self.method["parameters"]):
            types = []
            for param in self.method["parameters"]:
                if param["name"] == "&":  # fix for CppHeaderParser bug
                    param["type"] += " &"
                    param["reference"] = 1
                if param["unresolved"]:
                    const = "const " if param["constant"] or "const" in param["type"] else ""
                    raw_type = param["raw_type"]
                    if "const" in raw_type:  # fix for parsing error 'std::vector<double>const' (no space)
                        const = ""
                    ref = " &" if param["reference"] else ""
                    custom = CUSTOM_OVERLOAD_TYPES.get((param["method"]["parent"]["name"], raw_type))
                    if any(raw_type.startswith(base) for base in KEEP_DISAMIGUATION_TYPES_STARTSWITH):
                        pass
                    elif raw_type in self.method["parent"].get("template", ""):  # templated argument
                        pass
                    elif raw_type in EXPLICIT_IMPORTED_TYPES:  # todo: be more general...
                        pass
                    elif custom:
                        raw_type = custom
                    else:
                        raw_type = "%s::%s" % (class_name, raw_type)
                    type_ = const + raw_type + ref
                    if param.get("pointer"):
                        type_ += "*"
                else:
                    type_ = param["type"]
                if param.get("array_size"):
                    type_ += "[%s]" % param.get("array_size")
                types.append(type_)
            type_ = ", ".join(types)
        else:
            type_ = ""
        constant_method = ", py::const_" if self.method["const"] else ""
        disamb = "py::overload_cast<{type_}> (&{cls}::{name}{const})".format(type_=type_,
                                                                             cls=class_name,
                                                                             name=self.method["name"],
                                                                             const=constant_method)
        # ret_const = " const" if self.method["const"] else ""
        # ret = self.method["returns"]
        # disamb = "({ret}{ret_const} ({cls}::*)({type_})) ".format(cls=class_name,
        #                                                           type_=type_,
        #                                                           ret=ret,
        #                                                           ret_const=ret_const)
        return disamb

    def to_str(self, class_name, class_var_name):
        params = self.method["parameters"]
        has_templated_parameters = len(params) and any("T" in p["raw_type"] for p in params)
        if "operator" in self.method["name"]:
            message = "Operators not implemented (%s)" % (self.method["name"],)
            print("Warning: " + message)
            ret_val = "// " + message
        elif self.is_an_overload:
            disamb = self.make_disambiguation(class_name)
            ret_val = '{cls_var}.def("{name}", {disamb})'.format(cls_var=class_var_name,
                                                                 name=self.name,
                                                                 disamb=disamb)
        # elif has_templated_parameters:
        #     message = "templated arguments not implemented (%s)" % (self.method["name"],)
        #     print("Warning: " + message)
        #     ret_val = "// " + message
        #     """ Example of how to implement for kdtreeFlann:
        #     .def("nearest_k_search", []
        #     (Class &class_, const PointT &point, int k,
        #               std::vector<int> &k_indices, std::vector<float> &k_sqr_distances)
        #                {return class_.nearestKSearch(point, k, k_indices, k_sqr_distances);},
        #                                                  "point"_a,
        #                                                  "k"_a,
        #                                                  "k_indices"_a,
        #                                                  "k_sqr_distances"_a
        #                                                  )
        #                                                  """
        else:

            s = '{cls_var}.def("{name}", &{cls}::{cppname})'
            data = {"name": self.name,
                    "cls": class_name,
                    "cls_var": class_var_name,
                    "cppname": self.method["name"],
                    }
            ret_val = s.format(**data)
        return ret_val

    def __repr__(self):
        return "<Method %s>" % (self.name,)
