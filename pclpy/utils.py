from . import pcl
from copy import copy
import re
from boltons.funcutils import FunctionBuilder
from inflection import underscore
from functools import update_wrapper

point_cloud_types = [t for t in dir(pcl.PointCloud) if not t.startswith("__")]


def get_point_cloud_type(*point_clouds):
    return "_".join([type(pc).__name__ for pc in point_clouds])


def register_wrapper(base_class, get_type_from=("cloud",), instantiate_with=None):
    function_args = {"name": underscore(base_class.__name__.split(".")[-1]),
                     "module": "pclpy.api",
                     "filename": "pclpy.api",
                     "indent": 4,
                     }

    actual_class = [getattr(base_class, class_)
                    for class_ in dir(base_class)
                    if any(class_.startswith(p) for p in point_cloud_types)][0]

    setters = [s for s in dir(actual_class) if s.startswith("set")]
    kwonlyargs = []
    kwonlydefaults = {}
    args = []

    def clean_doc(doxygen):
        return doxygen.replace("\\brief", "").replace("*/", "").strip()

    doc = clean_doc(actual_class.__doc__)
    types = ", ".join(get_type_from)
    instantiation = ""
    if instantiate_with is not None:
        instantiation = ", ".join(instantiate_with)
    body = ["import pclpy",
            "pc_type = pclpy.utils.get_point_cloud_type(%s)" % types,
            "obj = getattr(%s, pc_type)(%s)" % (base_class.__name__, instantiation)]

    for setter in setters:
        setter_name = underscore(setter[3:])
        original_doc = getattr(actual_class, setter).__doc__
        if "brief" in original_doc:
            setter_doc = [line for line in original_doc.split("\n") if "brief" in line][0]
        else:
            setter_doc = ""
        setter_doc = clean_doc(setter_doc)
        doc += "\n%s: %s" % (setter_name, setter_doc)
        if setter_name == "input_cloud":
            args.append("cloud")
            body.append("obj.setInputCloud(cloud)")
        else:
            kwonlyargs.append(setter_name)
            kwonlydefaults[setter_name] = None
            body.append("if %s:" % (setter_name, ))
            body.append("    obj.%s(%s)" % (setter, setter_name))

    if "extract" in dir(actual_class):
        output_type = re.search(r"extract\(.+, .+: (.+)\)", actual_class.extract.__doc__).group(1)
        body.append("indices = %s()" % output_type)
        body.append("obj.extract(indices)")
        body.append("return indices")

    function_args["doc"] = doc
    function_args["body"] = "\n".join(body)
    function_args["args"] = args
    function_args["kwonlyargs"] = kwonlyargs
    function_args["kwonlydefaults"] = kwonlydefaults

    func = FunctionBuilder(**function_args).get_func()
    register_point_cloud_function(func)
    return func


def register_point_cloud_function(func):
    function_name = func.__name__
    for point_cloud_type in point_cloud_types:
        pc_type = getattr(pcl.PointCloud, point_cloud_type)
        setattr(pc_type, function_name, func)
    return func


def register_alias(function_name, func):
    f = update_wrapper(copy(func), func)
    f.__name__ = function_name
    register_point_cloud_function(f)
    return f
