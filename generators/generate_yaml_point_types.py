import os
from os.path import join
import re
from typing import List, Tuple
from collections import defaultdict

import yaml

from generators.config import MODULES_TO_BUILD
from generators.point_types_utils import PCL_POINT_TYPES
from generators.utils import parentheses_are_balanced

PCL_REPO_PATH = r"C:\github\pcl"

re_instantiate = re.compile(r"^PCL_INSTANTIATE\((.+?),(.+)\);?$")
re_product = re.compile(r"^PCL_INSTANTIATE_PRODUCT\((.+?),(.+)\);?$")

CLASS_EXCEPTIONS_ADD_NORMAL_T = ["RegionGrowing",
                                 "RegionGrowingRGB", ]

src_files = []
for m in MODULES_TO_BUILD:
    path = join(PCL_REPO_PATH, m, "src")
    files = []
    if os.path.isdir(path):
        files = os.listdir(path)
    src_files.append(files)


# http://stackoverflow.com/questions/4284991/parsing-nested-parentheses-in-python-grab-content-by-level
def parenthetic_contents(string):
    """Generate parenthesized contents in string as pairs (level, contents)."""
    stack = []
    for i, c in enumerate(string):
        if c == '(':
            stack.append(i)
        elif c == ')' and stack:
            start = stack.pop()
            yield (len(stack), string[start + 1: i])


def parse_point_list(point_list: str) -> List[str]:
    for group, types in PCL_POINT_TYPES.items():
        if group in point_list:
            point_list = point_list.replace(group, ",".join(types))
    point_list = point_list.replace(")(", ",").replace(")", "").replace("(", "")
    return point_list.split(",")


def parse_instantiation_line(line: str) -> Tuple[str, List[List[str]]]:
    line = line.replace(" ", "")
    single = re_instantiate.match(line)
    if single:
        class_name = single.group(1)
        point_list = single.group(2)
        types = parse_point_list(point_list)
        if class_name in CLASS_EXCEPTIONS_ADD_NORMAL_T:
            types = [types, ["Normal"]]
    else:
        product = re_product.match(line)
        class_name = product.group(1)
        point_list = product.group(2)
        types = [parse_point_list(info) for level, info in parenthetic_contents(point_list) if level == 0]
    return class_name, types


def get_instantiations(lines):
    inside_core_point_types_def = False
    skip_if = False
    complete_line = ""
    for line in lines:
        complete_line += line.strip()
        if complete_line.startswith("PCL_INSTANTIATE") and not skip_if:
            if parentheses_are_balanced(complete_line, "()"):
                yield parse_instantiation_line(complete_line)
            else:
                continue
        elif complete_line.startswith("#ifdef PCL_ONLY_CORE_POINT_TYPES"):
            inside_core_point_types_def = True
        elif inside_core_point_types_def and complete_line.startswith("#else"):
            inside_core_point_types_def = False
            skip_if = True
        elif skip_if and complete_line.startswith("#endif"):
            skip_if = False
        complete_line = ""


def main():
    all_classes = defaultdict(list)
    for files_list, module in zip(src_files, MODULES_TO_BUILD):
        cpp_files = [f for f in files_list if f.endswith("cpp")]
        for cpp in cpp_files:
            full = join(PCL_REPO_PATH, module, "src", cpp)
            for class_name, types in get_instantiations(open(full).readlines()):
                all_classes[class_name].append(types)
    yaml.dump(dict(all_classes), open("point_types_generated.yml", "w"))


if __name__ == '__main__':
    main()
