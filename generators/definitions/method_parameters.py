from collections import defaultdict
from typing import Dict, Set

from generators.config import GLOBAL_PCL_IMPORTS

all_default_types_by_namespace = defaultdict(set)
all_return_values = defaultdict(set)


def parameter_default_value(param):
    val = param.get("defaultValue", "")
    if val:
        namespace = param["method"]["namespace"]
        if any(val.startswith(g) for g in GLOBAL_PCL_IMPORTS):
            val = "pcl::" + val
        all_default_types_by_namespace[namespace].add(param["raw_type"])
        # fix for exponent and float values parsed with added spaces
        val = "=" + val.replace(" ", "")
        # fix for std::numeric_limits<unsignedshort>::max()
        val = val.replace("unsignedshort", "unsigned short")
    return val


def make_pybind_argument_list(cpp_parameters):
    if len(cpp_parameters) == 0:
        return ""
    names = ", ".join(['"%s"_a%s' % (p["name"], parameter_default_value(p)) for p in cpp_parameters])
    return ", " + names


def get_default_parameters_types() -> Dict[str, Set[str]]:
    default_parameters_types = defaultdict(set)
    for param in all_default_types_by_namespace:
        namespace = param["method"]["namespace"]
        default_parameters_types[namespace].add(param["raw_type"])

    return default_parameters_types
