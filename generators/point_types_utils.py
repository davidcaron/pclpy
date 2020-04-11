from functools import partial
from itertools import product
from typing import List, Tuple, Dict

import yaml

from generators.config import EXTERNAL_INHERITANCE, SKIPPED_INHERITANCE, GLOBAL_PCL_IMPORTS, KEEP_ASIS_TYPES
from generators.utils import parentheses_are_balanced, make_namespace_class

PCL_ALL_POINT_TYPES = {
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
        "(pcl::GASDSignature512)",
        "(pcl::GASDSignature984)",
        "(pcl::GASDSignature7992)",
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
        "(pcl::PointDEM)",
    ],

    "PCL_RGB_POINT_TYPES": [
        "(pcl::PointXYZRGBA)",
        "(pcl::PointXYZRGB)",
        "(pcl::PointXYZRGBL)",
        "(pcl::PointXYZRGBNormal)",
        "(pcl::PointSurfel)",
    ],

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
        "(pcl::PointDEM)",
    ],

    "PCL_XYZL_POINT_TYPES": [
        "(pcl::PointXYZL)",
        "(pcl::PointXYZRGBL)",
        "(pcl::PointXYZLNormal)",
    ],

    "PCL_NORMAL_POINT_TYPES": [
        "(pcl::Normal)",
        "(pcl::PointNormal)",
        "(pcl::PointXYZRGBNormal)",
        "(pcl::PointXYZINormal)",
        "(pcl::PointXYZLNormal)",
        "(pcl::PointSurfel)",
    ],

    "PCL_FEATURE_POINT_TYPES": [
        "(pcl::PFHSignature125)",
        "(pcl::PFHRGBSignature250)",
        "(pcl::PPFSignature)",
        "(pcl::CPPFSignature)",
        "(pcl::PPFRGBSignature)",
        "(pcl::NormalBasedSignature12)",
        "(pcl::FPFHSignature33)",
        "(pcl::VFHSignature308)",
        "(pcl::GASDSignature512)",
        "(pcl::GASDSignature984)",
        "(pcl::GASDSignature7992)",
        "(pcl::GRSDSignature21)",
        "(pcl::ESFSignature640)",
        "(pcl::BRISKSignature512)",
        "(pcl::Narf36)",
    ],

    "PCL_STATE_POINT_TYPES": [
        "(pcl::tracking::ParticleXYR)",
        "(pcl::tracking::ParticleXYZRPY)",
        "(pcl::tracking::ParticleXYZR)",
        "(pcl::tracking::ParticleXYRPY)",
        "(pcl::tracking::ParticleXYRP)",
    ],

    "PCL_POINT_CLOUD_TYPES": [
        "(pcl::PointCloud<pcl::PointXYZ>)",
        "(pcl::PointCloud<pcl::PointXYZI>)",
        "(pcl::PointCloud<pcl::PointXYZL>)",
        "(pcl::PointCloud<pcl::Label>)",
        "(pcl::PointCloud<pcl::PointXYZRGBA>)",
        "(pcl::PointCloud<pcl::PointXYZRGB>)",
        "(pcl::PointCloud<pcl::PointXYZRGBL>)",
        "(pcl::PointCloud<pcl::PointXYZHSV>)",
        "(pcl::PointCloud<pcl::PointXY>)",
        "(pcl::PointCloud<pcl::InterestPoint>)",
        "(pcl::PointCloud<pcl::Axis>)",
        "(pcl::PointCloud<pcl::Normal>)",
        "(pcl::PointCloud<pcl::PointNormal>)",
        "(pcl::PointCloud<pcl::PointXYZRGBNormal>)",
        "(pcl::PointCloud<pcl::PointXYZINormal>)",
        "(pcl::PointCloud<pcl::PointXYZLNormal>)",
        "(pcl::PointCloud<pcl::PointWithRange>)",
        "(pcl::PointCloud<pcl::PointWithViewpoint>)",
        "(pcl::PointCloud<pcl::MomentInvariants>)",
        "(pcl::PointCloud<pcl::PrincipalRadiiRSD>)",
        "(pcl::PointCloud<pcl::Boundary>)",
        "(pcl::PointCloud<pcl::PrincipalCurvatures>)",
        "(pcl::PointCloud<pcl::PFHSignature125>)",
        "(pcl::PointCloud<pcl::PFHRGBSignature250>)",
        "(pcl::PointCloud<pcl::PPFSignature>)",
        "(pcl::PointCloud<pcl::CPPFSignature>)",
        "(pcl::PointCloud<pcl::PPFRGBSignature>)",
        "(pcl::PointCloud<pcl::NormalBasedSignature12>)",
        "(pcl::PointCloud<pcl::FPFHSignature33>)",
        "(pcl::PointCloud<pcl::VFHSignature308>)",
        "(pcl::PointCloud<pcl::GASDSignature512>)",
        "(pcl::PointCloud<pcl::GASDSignature984>)",
        "(pcl::PointCloud<pcl::GASDSignature7992>)",
        "(pcl::PointCloud<pcl::GRSDSignature21>)",
        "(pcl::PointCloud<pcl::ESFSignature640>)",
        "(pcl::PointCloud<pcl::BRISKSignature512>)",
        "(pcl::PointCloud<pcl::Narf36>)",
        "(pcl::PointCloud<pcl::IntensityGradient>)",
        "(pcl::PointCloud<pcl::PointWithScale>)",
        "(pcl::PointCloud<pcl::PointSurfel>)",
        "(pcl::PointCloud<pcl::ShapeContext1980>)",
        "(pcl::PointCloud<pcl::UniqueShapeContext1960>)",
        "(pcl::PointCloud<pcl::SHOT352>)",
        "(pcl::PointCloud<pcl::SHOT1344>)",
        "(pcl::PointCloud<pcl::PointUV>)",
        "(pcl::PointCloud<pcl::ReferenceFrame>)",
        "(pcl::PointCloud<pcl::PointDEM>)",
    ],
}

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
        "(pcl::GASDSignature512)",
        "(pcl::GASDSignature984)",
        "(pcl::GASDSignature7992)",
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
        "(pcl::PointDEM)",
    ],

    "PCL_RGB_POINT_TYPES": [
        "(pcl::PointXYZRGBA)",
        # "(pcl::PointXYZRGB)",
        # "(pcl::PointXYZRGBL)",
        # "(pcl::PointXYZRGBNormal)",
        # "(pcl::PointSurfel)",
    ],

    "PCL_XYZ_POINT_TYPES": [
        "(pcl::PointXYZ)",
        "(pcl::PointXYZI)",
        # "(pcl::PointXYZL)",
        "(pcl::PointXYZRGBA)",
        # "(pcl::PointXYZRGB)",
        # "(pcl::PointXYZRGBL)",
        # "(pcl::PointXYZHSV)",
        # "(pcl::InterestPoint)",
        "(pcl::PointNormal)",
        # "(pcl::PointXYZRGBNormal)",
        # "(pcl::PointXYZINormal)",
        # "(pcl::PointXYZLNormal)",
        # "(pcl::PointWithRange)",
        # "(pcl::PointWithViewpoint)",
        # "(pcl::PointWithScale)",
        # "(pcl::PointSurfel)",
        # "(pcl::PointDEM)",
    ],

    "PCL_XYZL_POINT_TYPES": [
        "(pcl::PointXYZL)",
        # "(pcl::PointXYZRGBL)",
        # "(pcl::PointXYZLNormal)",
    ],

    "PCL_NORMAL_POINT_TYPES": [
        "(pcl::Normal)",
        "(pcl::PointNormal)",
        # "(pcl::PointXYZRGBNormal)",
        # "(pcl::PointXYZINormal)",
        # "(pcl::PointXYZLNormal)",
        # "(pcl::PointSurfel)",
    ],

    "PCL_FEATURE_POINT_TYPES": [
        "(pcl::PFHSignature125)",
        "(pcl::PFHRGBSignature250)",
        "(pcl::PPFSignature)",
        "(pcl::CPPFSignature)",
        "(pcl::PPFRGBSignature)",
        "(pcl::NormalBasedSignature12)",
        "(pcl::FPFHSignature33)",
        "(pcl::VFHSignature308)",
        "(pcl::GASDSignature512)",
        "(pcl::GASDSignature984)",
        "(pcl::GASDSignature7992)",
        "(pcl::GRSDSignature21)",
        "(pcl::ESFSignature640)",
        "(pcl::BRISKSignature512)",
        "(pcl::Narf36)",
    ],

    "PCL_STATE_POINT_TYPES": [
        "(pcl::tracking::ParticleXYR)",
        "(pcl::tracking::ParticleXYZRPY)",
        "(pcl::tracking::ParticleXYZR)",
        "(pcl::tracking::ParticleXYRPY)",
        "(pcl::tracking::ParticleXYRP)",
    ],

    "PCL_POINT_CLOUD_TYPES": [
        "(pcl::PointCloud<pcl::PointXYZ>)",
        "(pcl::PointCloud<pcl::PointXYZI>)",
        # "(pcl::PointCloud<pcl::PointXYZL>)",
        # "(pcl::PointCloud<pcl::Label>)",
        "(pcl::PointCloud<pcl::PointXYZRGBA>)",
        # "(pcl::PointCloud<pcl::PointXYZRGB>)",
        # "(pcl::PointCloud<pcl::PointXYZRGBL>)",
        # "(pcl::PointCloud<pcl::PointXYZHSV>)",
        # "(pcl::PointCloud<pcl::PointXY>)",
        # "(pcl::PointCloud<pcl::InterestPoint>)",
        # "(pcl::PointCloud<pcl::Axis>)",
        "(pcl::PointCloud<pcl::Normal>)",
        "(pcl::PointCloud<pcl::PointNormal>)",
        # "(pcl::PointCloud<pcl::PointXYZRGBNormal>)",
        # "(pcl::PointCloud<pcl::PointXYZINormal>)",
        # "(pcl::PointCloud<pcl::PointXYZLNormal>)",
        "(pcl::PointCloud<pcl::PointWithRange>)",
        "(pcl::PointCloud<pcl::PointWithViewpoint>)",
        "(pcl::PointCloud<pcl::MomentInvariants>)",
        "(pcl::PointCloud<pcl::PrincipalRadiiRSD>)",
        "(pcl::PointCloud<pcl::Boundary>)",
        "(pcl::PointCloud<pcl::PrincipalCurvatures>)",
        "(pcl::PointCloud<pcl::PFHSignature125>)",
        "(pcl::PointCloud<pcl::PFHRGBSignature250>)",
        "(pcl::PointCloud<pcl::PPFSignature>)",
        "(pcl::PointCloud<pcl::CPPFSignature>)",
        "(pcl::PointCloud<pcl::PPFRGBSignature>)",
        "(pcl::PointCloud<pcl::NormalBasedSignature12>)",
        "(pcl::PointCloud<pcl::FPFHSignature33>)",
        "(pcl::PointCloud<pcl::VFHSignature308>)",
        "(pcl::PointCloud<pcl::GASDSignature512>)",
        "(pcl::PointCloud<pcl::GASDSignature984>)",
        "(pcl::PointCloud<pcl::GASDSignature7992>)",
        "(pcl::PointCloud<pcl::GRSDSignature21>)",
        "(pcl::PointCloud<pcl::ESFSignature640>)",
        "(pcl::PointCloud<pcl::BRISKSignature512>)",
        "(pcl::PointCloud<pcl::Narf36>)",
        "(pcl::PointCloud<pcl::IntensityGradient>)",
        "(pcl::PointCloud<pcl::PointWithScale>)",
        "(pcl::PointCloud<pcl::PointSurfel>)",
        "(pcl::PointCloud<pcl::ShapeContext1980>)",
        "(pcl::PointCloud<pcl::UniqueShapeContext1960>)",
        "(pcl::PointCloud<pcl::SHOT352>)",
        "(pcl::PointCloud<pcl::SHOT1344>)",
        "(pcl::PointCloud<pcl::PointUV>)",
        "(pcl::PointCloud<pcl::ReferenceFrame>)",
        "(pcl::PointCloud<pcl::PointDEM>)",
    ],
}


def unpack_yaml_point_types(path, not_every_point_type=False):
    data = yaml.safe_load(open(path))
    for k, v in data.items():
        data[k] = unpack_point_types(v, not_every_point_type)
    return data


def filter_types(types):
    rgba = ["PointXYZRGBA"] if "PointXYZRGBA" in types else []
    xyz = ["PointXYZ"] if "PointXYZ" in types else []
    return list(set(types[:1] + rgba + xyz))


def unpack_point_types(types_info: List, not_every_point_type: bool):
    point_types = []
    for info in types_info:
        if isinstance(info[0], str):
            if not_every_point_type:
                info = filter_types(info)
            point_types += [(t,) for t in info]
        else:
            if not_every_point_type:
                info = list(map(filter_types, info))
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


def get_template_typenames_with_defaults(template: str) -> Dict[str, str]:
    template_typenames = {}
    splitters = ["typename", "class"]
    if template:
        template_string = template[template.find("<") + 1: template.rfind(">")]
        for name in splitters:
            splitted = template_string.split(name)
            for s in splitted:
                s = s.strip(", ")
                if "=" in s:
                    pos = s.find("=")
                    val = s[pos + 1:].strip(", ")
                    template_typenames[s[:pos].strip(", ")] = val
                elif s:
                    template_typenames[s] = ""
    template_typenames = {k: v for k, v in template_typenames.items() if not any(s in k for s in splitters)}
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
    # handle const prefix
    const = ""
    if type_.startswith("const "):
        type_ = type_.replace("const ", "", 1)
        const = "const "

    is_template_name = type_ in template_typenames_defaults
    typename_default = template_typenames_defaults.get(type_)
    has_default_and_replace = typename_default and replace_with_templated_typename

    if has_default_and_replace:
        type_ = typename_default

    keep_asis = any(type_.startswith(i) for i in KEEP_ASIS_TYPES)

    if keep_asis:
        pass
    elif is_template_name:
        pass
    elif not is_template_name or has_default_and_replace:
        if type_ in GLOBAL_PCL_IMPORTS:
            base_namespace = "pcl"

        is_external_inheritance = any(type_.startswith(i) for i in EXTERNAL_INHERITANCE)
        if not is_external_inheritance:
            namespaces = namespace_by_class_name and namespace_by_class_name.get(type_)
            if namespaces:
                base_namespace = namespaces[0]
            type_ = make_namespace_class(base_namespace, type_)

    type_ = const + type_
    return type_


def fix_cppheaderparser_bugs(inheritance: List[str]) -> List[str]:
    replace = {
        "constOctreeNode": "const OctreeNode",
    }
    fixed = []
    for i in inheritance:
        for k, v in replace.items():
            if k in i:
                i = i.replace(k, v)
        fixed.append(i)
    return fixed


def clean_inheritance(class_,
                      namespace_by_class_name=None,
                      replace_with_templated_typename=True,
                      formatted=False):
    inheritance = [i["class"] for i in class_["inherits"]]
    inheritance = fix_templated_inheritance(inheritance)
    inheritance = fix_cppheaderparser_bugs(inheritance)

    template_str = class_.get("template", "").replace("\n", "")
    template_typenames_defaults = get_template_typenames_with_defaults(template_str)

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
