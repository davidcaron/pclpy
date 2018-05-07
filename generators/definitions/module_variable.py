from CppHeaderParser import CppVariable

from generators.config import BASE_SUB_MODULE_NAME


def define_variable(var: CppVariable):
    variable_def = '{sub_name}.attr("%s") = %s;' % (var["name"], var["defaultValue"])
    return variable_def.format(sub_name=BASE_SUB_MODULE_NAME)
