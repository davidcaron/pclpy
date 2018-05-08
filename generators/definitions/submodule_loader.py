from inflection import camelize

from generators.config import common_includes, INDENT, declare_holder_type
from generators.utils import function_definition_name, include_make_opaque_vectors


def generate_loader(module, headers):
    lines = []
    a = lines.append
    a(common_includes)
    a(declare_holder_type)
    a(include_make_opaque_vectors(depth=1))
    a("")

    sub = camelize(module) if module != "base" else ""

    # add forward declarations
    for header in headers:
        name = function_definition_name(header_name=header)
        a("void define{sub}{name}Classes(py::module &);".format(sub=sub, name=name))

    a("\n")
    a("void define%sClasses(py::module &m) {" % camelize(module))

    # add submodule definition
    submodule_object_name = ("m_%s" % module) if module != "base" else "m"
    module_python = module if module != "2d" else "module_2d"
    if submodule_object_name != "m":
        module_creation = '{i}py::module {obj_name} = m.def_submodule("{module_python}", "Submodule {sub}");'
        a(module_creation.format(obj_name=submodule_object_name, module_python=module_python, sub=module, i=INDENT))

    # add function calls
    for header in headers:
        name = function_definition_name(header_name=header)
        function_call = "{i}define{sub}{name}Classes({sub_name});".format(i=INDENT,
                                                                          name=name,
                                                                          sub_name=submodule_object_name,
                                                                          sub=sub)
        a(function_call)
    a("}")

    return "\n".join(lines)
