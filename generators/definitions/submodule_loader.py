from os.path import join

import os

from inflection import camelize

from generators.config import PATH_MODULES, common_includes, INDENT
from generators.utils import function_definition_name


def generate_loader(module, headers):
    lines = [common_includes]
    a = lines.append
    a("PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);")
    a('#include "../make_opaque_vectors.hpp"')

    for header in headers:
        inc_name = '%s/%s' % (module, header) if module != "base" else header
        include_expression = '#include "%spp"' % (inc_name, )
        a(include_expression)
    a("\n")
    a("void define%sClasses(py::module &m) {" % camelize(module))

    # add submodule
    submodule_object_name = ("m_%s" % module) if module != "base" else "m"
    if submodule_object_name != "m":
        module_creation = '{i}py::module {obj_name} = m.def_submodule("{sub}", "Submodule {sub}");'
        a(module_creation.format(obj_name=submodule_object_name, sub=module, i=INDENT))

    # add function calls
    sub = camelize(module) if module != "base" else ""
    for header in headers:
        name = function_definition_name(header_name=header)
        function_call = "{i}define{sub}{name}Classes({sub_name});".format(i=INDENT,
                                                                            name=name,
                                                                            sub_name=submodule_object_name,
                                                                            sub=sub)
        a(function_call)
    a("}")

    return "\n".join(lines)
