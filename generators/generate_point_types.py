"""
PCL point types are generated using boost macros. So it's easier to manually bind them in pclpy.
"""

import re
from collections import OrderedDict
from os.path import join

from generators import config
from generators.config import PCL_BASE, INDENT


def get_point_types(header_path):
    point_types = OrderedDict()
    pt_type = None
    fields = []
    for line in open(header_path):
        if line.startswith("POINT_CLOUD_REGISTER_POINT_STRUCT"):
            pt_type = re.search("_POINT_STRUCT.*\((.+),", line).group(1).replace("_", "")
        elif line.startswith(")"):
            point_types[pt_type] = fields
            fields = []
            pt_type = None
        elif pt_type:
            # re_info = re.search("\((.+?)(\[\d+\])?, ?(.+), ?.+\)", line)
            re_info = re.search("\((.+), ?(.+), ?.+\)", line)
            type_, field_name = re_info.group(1), re_info.group(2)
            fields.append((type_, field_name))
    return point_types


def generate_class_point_type(module_name, point_name, fields, indent=INDENT):
    lines = []
    a = lines.append
    a('{i}py::class_<{point_name}, boost::shared_ptr<{point_name}>> ({module_name}, "{point_py_name}")')
    types_string = ", ".join(type_ for type_, _ in fields)
    args_string = ", ".join('"%s"_a' % name for _, name in fields)
    # if "[" in types_string:
    a('{i}{i}.def(py::init<>())')
    # else:
    #     a('{i}{i}.def(py::init<{types_string}>(), {args_string})')
    for type_, field_name in fields:
        write = "only" if "[" in type_ else "write"
        a('{i}{i}.def_read%s("%s", &{point_name}::%s)' % (write, field_name, field_name))
    data = {
        "i": indent,
        "types_string": types_string,
        "args_string": args_string,
        "point_name": point_name,
        "module_name": module_name,
        "point_py_name": point_name.replace("pcl::", ""),
    }
    lines[-1] += ";"
    return "\n".join(lines).format(**data)


def generate_point_types():
    header_path = join(PCL_BASE, "point_types.h")
    point_types = get_point_types(header_path)
    module_name = "m_pts"
    lines = [config.common_includes]
    a = lines.append
    a("void definePointTypes(py::module &m) {")
    module_def = '%spy::module %s = m.def_submodule("point_types", "Submodule for point types");'
    a(module_def % (INDENT, module_name))
    a("")
    for point_name, fields in point_types.items():
        a(generate_class_point_type(module_name, point_name, fields))
        a("")

    a("}")
    return "\n".join(lines)


def write_to_file(text):
    path = join(config.PATH_SRC, "point_types.hpp")
    open(path, "w").write(text)


def main():
    text = generate_point_types()
    write_to_file(text)


if __name__ == '__main__':
    main()
