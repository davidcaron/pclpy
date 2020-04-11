import re
from collections import OrderedDict
from itertools import chain, product
from typing import List

from CppHeaderParser import CppMethod, CppVariable

from generators.config import CUSTOM_OVERLOAD_TYPES, EXPLICIT_IMPORTED_TYPES, KEEP_ASIS_TYPES, \
    EXTERNAL_INHERITANCE, TEMPLATED_METHOD_TYPES, SPECIFIC_TEMPLATED_METHOD_TYPES, GLOBAL_PCL_IMPORTS
from generators.definitions.constructor import Constructor
from generators.definitions.method_parameters import make_pybind_argument_list
from generators.definitions.variable import Variable
from generators.point_types_utils import PCL_POINT_TYPES
from generators.utils import make_namespace_class, clean_doxygen


class Method:
    def __init__(self, method: CppMethod, is_an_overload=False, use_c_overload=False):
        """
        Generates definition for a method
        Example:
            cls.def("go", &Class::go);
        """
        self.cppmethod = method
        self.name = method["name"]
        self.is_an_overload = is_an_overload
        self.use_c_overload = use_c_overload
        self.templated_types = OrderedDict()
        self.needs_lambda_call = False

    def make_disambiguation(self, prefix, template_types=None):
        type_ = self.list_parameter_types(prefix, template_types)

        template = ("<%s>" % ", ".join([t[1] for t in template_types])) if template_types else ""
        template_keyword = "template " if template_types else ""
        constant_method = ", py::const_" if self.cppmethod["const"] else ""
        return_type = self.cppmethod["rtnType"].replace("inline", "").strip()
        if self.use_c_overload:
            disamb = "({return_type} (*)({type_}))(&{cls}{name})"
        else:
            disamb = "py::overload_cast<{type_}> (&{cls}{template_keyword}{name}{template}{const})"
        disamb = disamb.format(type_=type_,
                               cls=(prefix + "::") if prefix else "",
                               return_type=return_type,
                               name=self.cppmethod["name"],
                               template=template,
                               template_keyword=template_keyword,
                               const=constant_method)
        return disamb

    def list_parameter_types(self, prefix, template_types):
        if len(self.cppmethod["parameters"]):
            types = []
            for param in self.cppmethod["parameters"]:
                if param["name"] == "&":  # fix for CppHeaderParser bug
                    param["type"] += " &"
                    param["reference"] = 1
                type_ = param["type"] if not param["unresolved"] else param["raw_type"]
                
                if "boost::function<double(constdouble)" in type_:  # fix for CppHeaderParser bug
                    type_ = type_.replace('constdouble', 'const double')

                if template_types:
                    for name, template_type in template_types:
                        type_ = type_.replace(name, template_type)

                raw_type_no_pcl = param["raw_type"].replace("pcl::", "")
                if type_ in GLOBAL_PCL_IMPORTS:
                    type_ = make_namespace_class("pcl", type_)
                elif param["unresolved"]:
                    type_ = self.clean_unresolved_type(param, type_, prefix)
                elif raw_type_no_pcl in GLOBAL_PCL_IMPORTS:
                    # ensure pcl namespace in type
                    pos = type_.find(raw_type_no_pcl)
                    if not type_[pos - 5:pos] == "pcl::":
                        type_ = type_.replace(raw_type_no_pcl, "pcl::" + raw_type_no_pcl)

                if param["constant"] and not type_.startswith("const"):
                    type_ = "const " + type_
                if all(c in type_ for c in "<>"):
                    if type_.startswith("const"):
                        type_ = "const typename " + type_.replace("const ", "", 1)
                    else:
                        type_ = "typename " + type_
                if param["reference"] and "&" not in type_:
                    type_ += " &"

                if param.get("array_size"):
                    type_ += "[%s]" % param.get("array_size")

                types.append(type_)
            type_ = ", ".join(types)
        else:
            type_ = ""
        return type_

    def clean_unresolved_type(self, param, type_, prefix):
        const = "const " if param["constant"] or "const" in param["type"] else ""
        type_ = type_.replace("typename ", "")
        if "const" in type_:  # fix for CppHeaderParser parsing error 'std::vector<double>const' (no space)
            const = ""
        ref = " &" if param["reference"] else ""
        parent_name = param["method"].get("parent", {}).get("name")
        custom = None
        if parent_name:
            custom = CUSTOM_OVERLOAD_TYPES.get((parent_name, type_))
        type_no_template = type_[:type_.find("<")] if "<" in type_ else type_
        template_string = self.cppmethod.get("parent", {}).get("template", "")
        if type_ == "pcl::PCLPointCloud2::" and param["name"].startswith("Const"):
            type_ = type_ + param["name"]  # fix for CppHeaderParser bug
        elif type_.startswith("pcl::"):
            type_ = make_namespace_class("pcl", type_)
        elif any(type_.startswith(base) for base in KEEP_ASIS_TYPES):
            pass
        elif type_ in template_string:  # templated argument
            pass
        elif type_no_template in EXPLICIT_IMPORTED_TYPES:
            pass
        elif type_no_template in GLOBAL_PCL_IMPORTS:
            pass
        elif custom:
            type_ = custom
        elif any(type_.startswith(t) for t in EXTERNAL_INHERITANCE):
            pass
        elif prefix == "pcl":
            type_ = make_namespace_class("pcl", type_)
        else:
            type_ = "%s::%s" % (prefix, type_)
        for global_import in GLOBAL_PCL_IMPORTS:
            if re.search(r"[^a-zA-Z:]%s" % global_import, type_):
                type_ = type_.replace(global_import, "pcl::" + global_import)
        type_ = const + type_ + ref
        if param.get("pointer"):
            type_ = type_.strip() + "*"
        type_ = type_.replace("constpcl::", "const pcl::")  # parser error for this expression
        return type_

    def static_value(self):
        return "_static" if self.cppmethod["static"] else ""

    def disambiguated_function_call(self, cls_var, disamb, args):
        doc = clean_doxygen(self.cppmethod.get("doxygen", ""))
        return '{cls_var}.def{static}("{name}", {disamb}{args}, R"({doc})")'.format(cls_var=cls_var,
                                                                                    name=self.name,
                                                                                    disamb=disamb,
                                                                                    static=self.static_value(),
                                                                                    args=args,
                                                                                    doc=doc)

    def to_str(self, prefix, class_var_name):
        params = self.cppmethod["parameters"]
        doc = clean_doxygen(self.cppmethod.get("doxygen", ""))
        args = make_pybind_argument_list(params)
        if "operator" in self.cppmethod["name"]:
            message = "Operators not implemented (%s)" % (self.cppmethod["name"],)
            ret_val = "// " + message
        elif self.needs_lambda_call:
            message = "Non templated function disambiguation not implemented (%s)" % (self.cppmethod["name"],)
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
        elif any("**" in param["type"].replace(" ", "") for param in params):
            message = "Double pointer arguments are not supported by pybind11 (%s)" % (self.cppmethod["name"],)
            ret_val = "// " + message
        elif self.is_boost_function_callback():
            s = '{cls_var}.def("{name}", []({cls} &v, {types} &cb) {ob} v.{name}(cb); {cb}, R"({doc})")'
            types = self.list_parameter_types(prefix, template_types=None)
            types = types.replace("boost::function", "std::function")
            data = {"name": self.name,
                    "cls": prefix,
                    "cls_var": class_var_name,
                    "types": types,
                    "ob": "{ob}",  # will get formatted later
                    "cb": "{cb}",
                    "doc": doc,
                    }
            ret_val = s.format(**data)
        elif self.templated_types:
            return_values = []
            names = list(self.templated_types.keys())
            types = list(self.templated_types.values())
            for types_combination in product(*types):
                template_types = tuple(zip(names, types_combination))
                disamb = self.make_disambiguation(prefix, template_types=template_types)
                return_values.append(self.disambiguated_function_call(class_var_name, disamb, args))
            ret_val = return_values
        elif self.is_an_overload:
            disamb = self.make_disambiguation(prefix)
            ret_val = self.disambiguated_function_call(class_var_name, disamb, args)
        else:
            s = '{cls_var}.def{static}("{name}", &{cls}{name}{args}, R"({doc})")'
            data = {"name": self.name,
                    "static": self.static_value(),
                    "cls": (prefix + "::") if prefix else "",
                    "cls_var": class_var_name,
                    "args": args,
                    "doc": doc,
                    }
            ret_val = s.format(**data)
        return ret_val

    def is_boost_function_callback(self):
        callback = "Callback" in self.name
        if callback:
            single_argument = len(self.cppmethod["parameters"]) == 1
            if single_argument:
                if self.cppmethod["parameters"][0]["type"].startswith("boost::function"):
                    return True
        return False

    def __repr__(self):
        return "<Method %s>" % (self.name,)


def flag_templated_methods(methods: List[Method]):
    for method in methods:
        method_name = method.cppmethod["name"]
        if "operator" in method_name:
            continue
        template = method.cppmethod["template"]
        if template:
            class_name = method.cppmethod["parent"]["name"]
            pos = template.find("<")
            type_names = filter_template_types(template[pos + 1:-1], keep=["typename", "class"])
            for type_name, types in template_types_generator(type_names, class_name, method_name):
                method.templated_types[type_name] = types


def template_types_generator(type_names, header_or_class_name, method_name):
    method_key = (header_or_class_name, method_name, type_names)
    all_methods_key = (header_or_class_name, "", type_names)
    specific = SPECIFIC_TEMPLATED_METHOD_TYPES
    pcl_point_types = specific.get(method_key, specific.get(all_methods_key))
    if not pcl_point_types:
        pcl_point_types = [TEMPLATED_METHOD_TYPES.get(type_name) for type_name in type_names]

    for type_name, pcl_types in zip(type_names, pcl_point_types):
        if not pcl_types:
            attrs = (type_name, method_name, header_or_class_name)
            message = "Templated method name not implemented (name=%s method=%s header=%s)"
            raise NotImplementedError(message % attrs)
        if isinstance(pcl_types, list):
            types = pcl_types
        elif pcl_types in PCL_POINT_TYPES:
            types = [t[1:-1] for t in PCL_POINT_TYPES[pcl_types]]  # remove parentheses
        else:
            raise ValueError
        yield type_name, types


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
    constructors_methods = [m for m in methods if m["constructor"] and
                            not is_copy_constructor(m) and m["name"] == m["parent"]["name"]]
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


def filter_template_types(template_string, keep=None, keep_all=False):
    if keep is None:
        keep = ["typename", "class", "unsigned"]
    if not template_string:
        return tuple()
    types = template_string.split(", ")
    types = [s.strip().split(" ")[1] for s in types if keep_all or any(k in s for k in keep)]
    return tuple(types)


def is_copy_constructor(method):
    name = method["name"]
    params = method["parameters"]
    if params and name in params[0]["type"]:
        try:
            in_doxygen = "Copy constructor" in method["doxygen"].split("\n")[0]
        except KeyError:
            in_doxygen = False

        # some doxygen strings are wrong, we need to use the number of parameters also
        if in_doxygen and len(params) >= 2:
            return False
        if len(params) == 1 or in_doxygen:
            return True
    return False
