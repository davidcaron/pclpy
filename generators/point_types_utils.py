from itertools import product
from functools import partial
from typing import List, Tuple

import yaml

from generators.config import EXTERNAL_INHERITANCE, SKIPPED_INHERITANCE, GLOBAL_PCL_IMPORTS, KEEP_ASIS_TYPES_STARTSWITH
from generators.utils import parentheses_are_balanced, make_namespace_class

PCL_POINT_TYPES = {
    "PCL_POINT_TYPES": [
        "(pcl::PointXYZ)",
        "(pcl::PointXYZI)",
        "(pcl::PointXYZL)",
        "(pcl::Label)",
        "(pcl::PointXYZRGBA)",
        "(pcl::PointXYZRGB)",
        "(pcl::PointXYZRGBL)",
        "(pcl::PointXYZHSV)",
        "(pcl::PointXY)",
        "(pcl::InterestPoint)",
        "(pcl::Axis)",
        "(pcl::Normal)",
        "(pcl::PointNormal)",
        "(pcl::PointXYZRGBNormal)",
        "(pcl::PointXYZINormal)",
        "(pcl::PointXYZLNormal)",
        "(pcl::PointWithRange)",
        "(pcl::PointWithViewpoint)",
        "(pcl::MomentInvariants)",
        "(pcl::PrincipalRadiiRSD)",
        "(pcl::Boundary)",
        "(pcl::PrincipalCurvatures)",
        "(pcl::PFHSignature125)",
        "(pcl::PFHRGBSignature250)",
        "(pcl::PPFSignature)",
        "(pcl::CPPFSignature)",
        "(pcl::PPFRGBSignature)",
        "(pcl::NormalBasedSignature12)",
        "(pcl::FPFHSignature33)",
        "(pcl::VFHSignature308)",
        "(pcl::GRSDSignature21)",
        "(pcl::ESFSignature640)",
        "(pcl::BRISKSignature512)",
        "(pcl::Narf36)",
        "(pcl::IntensityGradient)",
        "(pcl::PointWithScale)",
        "(pcl::PointSurfel)",
        "(pcl::ShapeContext1980)",
        "(pcl::UniqueShapeContext1960)",
        "(pcl::SHOT352)",
        "(pcl::SHOT1344)",
        "(pcl::PointUV)",
        "(pcl::ReferenceFrame)",
        "(pcl::PointDEM)"],

    "PCL_RGB_POINT_TYPES": [
        "(pcl::PointXYZRGBA)",
        "(pcl::PointXYZRGB)",
        "(pcl::PointXYZRGBL)",
        "(pcl::PointXYZRGBNormal)",
        "(pcl::PointSurfel)"],

    "PCL_XYZ_POINT_TYPES": [
        "(pcl::PointXYZ)",
        "(pcl::PointXYZI)",
        "(pcl::PointXYZL)",
        "(pcl::PointXYZRGBA)",
        "(pcl::PointXYZRGB)",
        "(pcl::PointXYZRGBL)",
        "(pcl::PointXYZHSV)",
        "(pcl::InterestPoint)",
        "(pcl::PointNormal)",
        "(pcl::PointXYZRGBNormal)",
        "(pcl::PointXYZINormal)",
        "(pcl::PointXYZLNormal)",
        "(pcl::PointWithRange)",
        "(pcl::PointWithViewpoint)",
        "(pcl::PointWithScale)",
        "(pcl::PointSurfel)",
        "(pcl::PointDEM)"],

    "PCL_XYZL_POINT_TYPES": [
        "(pcl::PointXYZL)",
        "(pcl::PointXYZRGBL)",
        "(pcl::PointXYZLNormal)"],

    "PCL_NORMAL_POINT_TYPES": [
        "(pcl::Normal)",
        "(pcl::PointNormal)",
        "(pcl::PointXYZRGBNormal)",
        "(pcl::PointXYZINormal)",
        "(pcl::PointXYZLNormal)",
        "(pcl::PointSurfel)"],

    "PCL_FEATURE_POINT_TYPES": [
        "(pcl::PFHSignature125)",
        "(pcl::PFHRGBSignature250)",
        "(pcl::PPFSignature)",
        "(pcl::CPPFSignature)",
        "(pcl::PPFRGBSignature)",
        "(pcl::NormalBasedSignature12)",
        "(pcl::FPFHSignature33)",
        "(pcl::VFHSignature308)",
        "(pcl::GRSDSignature21)",
        "(pcl::ESFSignature640)",
        "(pcl::BRISKSignature512)",
        "(pcl::Narf36)"],

    "PCL_STATE_POINT_TYPES": [
        "(pcl::tracking::ParticleXYR)",
        "(pcl::tracking::ParticleXYZRPY)",
        "(pcl::tracking::ParticleXYZR)",
        "(pcl::tracking::ParticleXYRPY)",
        "(pcl::tracking::ParticleXYRP)"],
}


def unpack_yaml_point_types(path):
    data = yaml.load(open(path))
    for k, v in data.items():
        data[k] = unpack_point_types(v)
    return data


def unpack_point_types(types_info: List):
    point_types = []
    for info in types_info:
        if isinstance(info[0], str):
            point_types += [(t,) for t in info]
        else:
            point_types += list(map(tuple, product(*info)))
    return point_types


def fix_templated_inheritance(inherits):
    while True:
        for n, i in enumerate(inherits[:]):
            if not parentheses_are_balanced(i, "<>"):
                inherits = inherits[:n] + ["%s, %s" % (i, inherits[n + 1])] + inherits[n + 2:]
                break
        else:
            break
    return inherits


def get_template_typenames_with_defaults(class_):
    template = class_.get("template", "").replace("\n", "")
    template_typenames = {}
    if template:
        template_string = template[template.find("<") + 1: template.rfind(">") - 1]
        splitted = template_string.split("typename")
        for s in splitted:
            if "=" in s:
                pos = s.find("=")
                val = s[pos + 1:].strip(", ")
                template_typenames[s[:pos].strip(", ")] = val
    return template_typenames


def get_class_namespace(class_name):
    assert "::" in class_name
    return class_name[class_name.rfind("::"):]


def split_templated_class_name(class_name: str) -> Tuple:
    """
    Example:
        "OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::Ptr"
        ("OctreePointCloud", (PointT, LeafContainerT, BranchContainerT), "::Ptr")
    """
    template_types = tuple()
    pos = class_name.find("<", 1)
    pos_end = class_name.rfind(">")
    is_templated = pos > 0
    end_pos_basename = pos if is_templated else None
    class_base_name = class_name[:end_pos_basename]
    after_template = class_name[pos_end + 1:] if is_templated else ""
    if is_templated:
        template_types = tuple([s.strip() for s in class_name[pos + 1:pos_end].split(",")])
    return class_base_name, template_types, after_template


def format_type_with_namespace(type_,
                               base_namespace,
                               namespace_by_class_name,
                               template_typenames_defaults,
                               replace_with_templated_typename):
    typename_default = template_typenames_defaults.get(type_)
    has_default_and_replace = typename_default and replace_with_templated_typename

    if has_default_and_replace:
        type_ = typename_default

    keep_asis = any(type_.startswith(i) for i in KEEP_ASIS_TYPES_STARTSWITH)

    if not keep_asis and not typename_default or has_default_and_replace:
        if type_ in GLOBAL_PCL_IMPORTS:
            base_namespace = "pcl"

        is_external_inheritance = any(type_.startswith(i) for i in EXTERNAL_INHERITANCE)
        if not is_external_inheritance:
            namespaces = namespace_by_class_name and namespace_by_class_name.get(type_)
            if namespaces:
                base_namespace = namespaces[0]
            type_ = make_namespace_class(base_namespace, type_)

    return type_


def clean_inheritance(class_,
                      namespace_by_class_name=None,
                      replace_with_templated_typename=True,
                      formatted=False):
    inheritance = [i["class"] for i in class_["inherits"]]
    inheritance = fix_templated_inheritance(inheritance)

    template_typenames_defaults = get_template_typenames_with_defaults(class_)

    args = {
        "base_namespace": class_["namespace"],
        "namespace_by_class_name": namespace_by_class_name,
        "template_typenames_defaults": template_typenames_defaults,
        "replace_with_templated_typename": replace_with_templated_typename,
    }
    format_namespace = partial(format_type_with_namespace, **args)

    for inherits in inheritance:
        if any(inherits.startswith(i) for i in SKIPPED_INHERITANCE):
            continue

        class_base_name, template_types, after_template = split_templated_class_name(inherits)

        class_base_name = format_namespace(class_base_name)
        template_types = tuple(map(format_namespace, template_types))
        if formatted:
            template = ("<%s>" % ", ".join(template_types)) if template_types else ""
            yield "%s%s%s" % (class_base_name, template, after_template)
        else:
            yield (class_base_name, template_types)
