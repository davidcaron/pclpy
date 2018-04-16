from collections import OrderedDict
from itertools import chain, product

from generators.point_types_utils import PCL_POINT_TYPES
from generators.definitions.property import make_properties_split_overloads

from generators.definitions.constructor import Constructor
from generators.definitions.variable import Variable
from typing import List

from CppHeaderParser import CppMethod, CppVariable

from generators.config import CUSTOM_OVERLOAD_TYPES, EXPLICIT_IMPORTED_TYPES, KEEP_DISAMIGUATION_TYPES_STARTSWITH, \
    EXTERNAL_INHERITANCE, TEMPLATED_METHOD_TYPES, SPECIFIC_TEMPLATED_METHOD_TYPES, GLOBAL_PCL_IMPORTS
from generators.definitions.method_parameters import make_pybind_argument_list
from generators.utils import make_namespace_class


class Method:
    def __init__(self, method: CppMethod, is_an_overload=False):
        """
        Generates definition for a method
        Example:
            .def("go", &Class::go)
        """
        self.cppmethod = method
        self.name = method["name"]
        self.is_an_overload = is_an_overload
        self.templated_types = OrderedDict()
        self.needs_lambda_call = False

    def make_disambiguation(self, class_name, template_types=None):
        if len(self.cppmethod["parameters"]):
            types = []
            for param in self.cppmethod["parameters"]:
                if param["name"] == "&":  # fix for CppHeaderParser bug
                    param["type"] += " &"
                    param["reference"] = 1

                type_ = param["type"] if not param["unresolved"] else param["raw_type"]

                if template_types:
                    for name, template_type in template_types:
                        type_ = type_.replace(name, template_type)

                if param["unresolved"]:
                    const = "const " if param["constant"] or "const" in param["type"] else ""
                    type_ = type_.replace("typename ", "")
                    if "const" in type_:  # fix for parsing error 'std::vector<double>const' (no space)
                        const = ""
                    ref = " &" if param["reference"] else ""
                    custom = CUSTOM_OVERLOAD_TYPES.get((param["method"]["parent"]["name"], type_))
                    type_no_template = type_[:type_.find("<")] if "<" in type_ else type_
                    if type_.startswith("pcl::"):
                        type_ = make_namespace_class("pcl", type_)
                    elif any(type_.startswith(base) for base in KEEP_DISAMIGUATION_TYPES_STARTSWITH):
                        pass
                    elif type_ in self.cppmethod["parent"].get("template", ""):  # templated argument
                        pass
                    elif type_no_template in EXPLICIT_IMPORTED_TYPES:
                        pass
                    elif type_no_template in GLOBAL_PCL_IMPORTS:
                        pass
                    elif custom:
                        type_ = custom
                    elif any(type_.startswith(t) for t in EXTERNAL_INHERITANCE):
                        pass
                    else:
                        type_ = "%s::%s" % (class_name, type_)

                    for global_pcl in GLOBAL_PCL_IMPORTS:
                        pos = type_.find(global_pcl)
                        if not type_[pos - 5:pos] == "pcl::":
                            type_ = type_.replace(global_pcl, "pcl::" + global_pcl)

                    type_ = const + type_ + ref
                    if param.get("pointer"):
                        type_ = type_.strip() + "*"
                    type_ = type_.replace("constpcl::", "const pcl::")  # parser error for this expression
                else:
                    type_ = param["type"]

                if param.get("array_size"):
                    type_ += "[%s]" % param.get("array_size")

                types.append(type_)
            type_ = ", ".join(types)
        else:
            type_ = ""

        template = ("<%s>" % ", ".join([t[1] for t in template_types])) if template_types else ""
        constant_method = ", py::const_" if self.cppmethod["const"] else ""
        disamb = "py::overload_cast<{type_}> (&{cls}::{name}{template}{const})"
        disamb = disamb.format(type_=type_,
                               cls=class_name,
                               name=self.cppmethod["name"],
                               template=template,
                               const=constant_method)
        return disamb

    def static_value(self):
        return "_static" if self.cppmethod["static"] else ""

    def disambiguated_function_call(self, cls_var, disamb, args):
        return '{cls_var}.def{static}("{name}", {disamb}{args})'.format(cls_var=cls_var,
                                                                        name=self.name,
                                                                        disamb=disamb,
                                                                        static=self.static_value(),
                                                                        args=args)

    def to_str(self, class_name, class_var_name):
        params = self.cppmethod["parameters"]
        args = make_pybind_argument_list(params)
        if "operator" in self.cppmethod["name"]:
            message = "Operators not implemented (%s)" % (self.cppmethod["name"],)
            print("Warning: " + message)
            ret_val = "// " + message
        elif self.needs_lambda_call:
            message = "Non templated function disambiguation not implemented (%s)" % (self.cppmethod["name"],)
            print("Warning: " + message)
            ret_val = "// " + message
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
        elif self.templated_types:
            return_values = []
            names = list(self.templated_types.keys())
            types = list(self.templated_types.values())
            for types_combination in product(*types):
                template_types = tuple(zip(names, types_combination))
                disamb = self.make_disambiguation(class_name, template_types=template_types)
                return_values.append(self.disambiguated_function_call(class_var_name, disamb, args))
            ret_val = return_values
        elif self.is_an_overload:
            disamb = self.make_disambiguation(class_name)
            ret_val = self.disambiguated_function_call(class_var_name, disamb, args)
        else:
            s = '{cls_var}.def{static}("{name}", &{cls}::{cppname}{args})'
            data = {"name": self.name,
                    "static": self.static_value(),
                    "cls": class_name,
                    "cls_var": class_var_name,
                    "cppname": self.cppmethod["name"],
                    "args": args,
                    }
            ret_val = s.format(**data)
        return ret_val

    def __repr__(self):
        return "<Method %s>" % (self.name,)


def flag_templated_methods(methods: List[Method]):
    for method in methods:
        method_name = method.cppmethod["name"]
        if "operator" in method_name:
            continue
        template = method.cppmethod["template"]
        if template:
            pos = template.find("<")
            type_names = filter_template_types(template[pos + 1:-1], keep=["typename"])

            class_name = method.cppmethod["parent"]["name"]
            method_key = (class_name, method_name, type_names)
            all_methods_key = (class_name, "", type_names)
            specific = SPECIFIC_TEMPLATED_METHOD_TYPES
            pcl_point_types = specific.get(all_methods_key, specific.get(method_key))
            if not pcl_point_types:
                pcl_point_types = [TEMPLATED_METHOD_TYPES.get(type_name) for type_name in type_names]

            for type_name, pcl_types in zip(type_names, pcl_point_types):
                if not pcl_types:
                    attrs = (type_name, method_name, class_name)
                    message = "Templated method name not implemented (name=%s method=%s class=%s)"
                    raise NotImplementedError(message % attrs)
                if isinstance(pcl_types, list):
                    types = pcl_types
                elif pcl_types in PCL_POINT_TYPES:
                    types = [t[1:-1] for t in PCL_POINT_TYPES[pcl_types]]  # remove parentheses
                else:
                    raise ValueError
                method.templated_types[type_name] = types


def flag_overloaded_methods(methods: List[Method], needs_overloading: List[str] = None):
    templated_method_names = [m.cppmethod["name"] for m in methods if m.templated_types]
    # flag methods that need to be called with a lambda (same name and same parameters as a templated method)
    for method in methods:
        name_ = method.cppmethod["name"]
        method.is_an_overload = True
        if method.templated_types:
            pass
        elif name_ in templated_method_names:
            method.needs_lambda_call = True
        elif name_ in needs_overloading:
            pass
        else:
            method.is_an_overload = False


def filter_static_and_non_static_methods(methods: List[Method]):
    static = set(m.cppmethod["name"] for m in methods if m.cppmethod["static"])
    not_static = set(m.cppmethod["name"] for m in methods if not m.cppmethod["static"])
    both = static & not_static
    filtered = []
    for method in methods:
        name_ = method.cppmethod["name"]
        is_static = method.cppmethod["static"]
        if name_ in both and is_static:
            print("Warning: Overloading a method with both static "
                  "and instance methods is not supported by pybind11 (%s)" % name_)
            continue
        filtered.append(method)
    return filtered


def split_methods_by_type(methods: List[CppMethod],
                          class_variables: List[CppVariable],
                          needs_overloading: List[str]):
    constructors_methods = [m for m in methods if m["constructor"] and not is_copy_constructor(m)]
    copy_const = [m for m in methods if m["constructor"] and is_copy_constructor(m)]
    destructors = [m for m in methods if m["destructor"]]
    setters = [m for m in methods if m["name"].startswith("set")]
    getters = [m for m in methods if m["name"].startswith("get")]

    identified_methods = chain(constructors_methods, copy_const, destructors, setters, getters)
    identified_methods_line_numbers = set([m["line_number"] for m in identified_methods])
    other_methods = [m for m in methods if m["line_number"] not in identified_methods_line_numbers]

    others = [Method(m) for m in chain(other_methods, setters, getters)]

    flag_templated_methods(others)
    flag_overloaded_methods(others, needs_overloading)
    others = filter_static_and_non_static_methods(others)

    variables = list(map(Variable, class_variables))
    constructors = list(map(Constructor, constructors_methods))

    return constructors, variables, others


def filter_template_types(template_string, keep=None):
    if keep is None:
        keep = ["typename", "class", "unsigned"]
    if not template_string:
        return []
    types = template_string.split(", ")
    types = [s.strip().split(" ")[1] for s in types if any(k in s for k in keep)]  # and "=" not in s]
    return tuple(types)


def is_copy_constructor(method):
    name = method["name"]
    params = method["parameters"]
    return len(params) == 1 and name in params[0]["type"]
