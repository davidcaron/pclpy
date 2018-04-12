all_parameters_with_default = []


def parameter_default_value(param):
    val = param.get("defaultValue", "")
    if val:
        all_parameters_with_default.append(param)
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
