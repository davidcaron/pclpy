from itertools import chain

from generators.utils import split_overloads
from generators.definitions.property import make_properties_split_overloads

from generators.definitions.constructor import Constructor
from generators.definitions.variable import Variable
from inflection import underscore
from typing import List

from CppHeaderParser import CppMethod, CppVariable

from generators.constants import CUSTOM_OVERLOAD_TYPES, EXPLICIT_IMPORTED_TYPES, KEEP_DISAMIGUATION_TYPES_STARTSWITH, \
    TEMPLATED_METHOD_TYPES
from generators.point_types_utils import PCL_POINT_TYPES


class Method:
    def __init__(self, method: CppMethod, is_an_overload=False):
        """
        Generates definition for a method
        Example:
            .def("go", &Class::go)
        """
        self.cppmethod = method
        self.name = underscore(method["name"])
        self.is_an_overload = is_an_overload
        self.templated_types = []
        self.needs_lambda_call = False

    def make_disambiguation(self, class_name):
        if len(self.cppmethod["parameters"]):
            types = []
            for param in self.cppmethod["parameters"]:
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
                    elif raw_type in self.cppmethod["parent"].get("template", ""):  # templated argument
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
        constant_method = ", py::const_" if self.cppmethod["const"] else ""
        disamb = "py::overload_cast<{type_}> (&{cls}::{name}{const})".format(type_=type_,
                                                                             cls=class_name,
                                                                             name=self.cppmethod["name"],
                                                                             const=constant_method)
        # ret_const = " const" if self.method["const"] else ""
        # ret = self.method["returns"]
        # disamb = "({ret}{ret_const} ({cls}::*)({type_})) ".format(cls=class_name,
        #                                                           type_=type_,
        #                                                           ret=ret,
        #                                                           ret_const=ret_const)
        return disamb

    def to_str(self, class_name, class_var_name):
        params = self.cppmethod["parameters"]
        has_templated_parameters = len(params) and any("T" in p["raw_type"] for p in params)
        if "operator" in self.cppmethod["name"]:
            message = "Operators not implemented (%s)" % (self.cppmethod["name"],)
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
                    "cppname": self.cppmethod["name"],
                    }
            ret_val = s.format(**data)
        return ret_val

    def __repr__(self):
        return "<Method %s>" % (self.name,)


def flag_templated_overloads(other_overloads: List[Method]):
    for method in other_overloads:
        template = method.cppmethod["template"]
        if template:
            pos = template.find("<")
            type_names = filter_template_types(template[pos + 1:-1], keep=["typename"])
            for name in type_names:
                pcl_point_types = TEMPLATED_METHOD_TYPES.get(name)
                if not pcl_point_types:
                    raise NotImplementedError("Templated method name not implemented:%s" % pcl_point_types)
                method.templated_types = [t[1:-1] for t in PCL_POINT_TYPES[pcl_point_types]]  # remove parentheses
    templated_methods = [m for m in other_overloads if m.templated_types]

    # flag methods that need to be called with a lambda (same name and same parameters as a templated method)
    for method in other_overloads:
        if method in templated_methods:
            continue
        n_parameters = len(method.cppmethod["parameters"])
        for m in templated_methods:
            if n_parameters == len(m.cppmethod["parameters"]):
                method.needs_lambda_call = True


def split_methods_by_type(methods: List[CppMethod],
                          class_variables: List[CppVariable],
                          needs_overloading: List[str]):
    # methods_with_name = [m for m in methods if m["name"]]  # CppHeaderParser bug
    constructors_methods = [m for m in methods if m["constructor"] and not is_copy_constructor(m)]
    copy_const = [m for m in methods if m["constructor"] and is_copy_constructor(m)]
    destructors = [m for m in methods if m["destructor"]]
    setters = [m for m in methods if m["name"].startswith("set")]
    getters = [m for m in methods if m["name"].startswith("get")]

    identified_methods = chain(constructors_methods, copy_const, destructors, setters, getters)
    identified_methods_line_numbers = set([m["line_number"] for m in identified_methods])
    others_methods = [m for m in methods if m["line_number"] not in identified_methods_line_numbers]

    others_overloads, others_unique = split_overloads(others_methods, needs_overloading)
    others_overloads = [Method(m, is_an_overload=True) for m in others_overloads]
    others_unique = [Method(m) for m in others_unique]

    flag_templated_overloads(others_overloads)

    properties_overloads, properties = make_properties_split_overloads(setters, getters)
    properties_overloads = [Method(m, is_an_overload=True) for m in properties_overloads]

    variables = list(map(Variable, class_variables))
    constructors = list(map(Constructor, constructors_methods))

    others = others_overloads + others_unique + properties_overloads

    return constructors, properties, variables, others


def filter_template_types(template_string, keep=None):
    if keep is None:
        keep = ["typename", "class", "unsigned"]
    if not template_string:
        return []
    types = template_string.split(", ")
    types = [s.strip().split(" ")[1] for s in types if any(k in s for k in keep)]  # and "=" not in s]
    return types


def is_copy_constructor(method):
    name = method["name"]
    params = method["parameters"]
    return len(params) == 1 and name in params[0]["type"]
