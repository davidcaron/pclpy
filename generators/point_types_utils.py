from collections import defaultdict, deque, namedtuple
from itertools import product
from typing import Dict, List

from generators.constants import IGNORE_INHERITED_INSTANTIATIONS, INHERITED_TEMPLATED_TYPES_FILTER, \
    EXTERNAL_INHERITANCE, SKIPPED_INHERITANCE
from generators.utils import parentheses_are_balanced, make_namespace_class

from CppHeaderParser import CppClass

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


def unpack_point_types(types_info: List):
    point_types = []
    for info in types_info:
        if isinstance(info[0], str):
            point_types += [(t,) for t in info]
        else:
            point_types += list(map(tuple, product(*info)))
    return point_types


class DependencyTree:
    def __init__(self, classes: List[CppClass]):
        self.namespace_by_class_name = defaultdict(list)
        for c in classes:
            self.namespace_by_class_name[c["name"]].append(c["namespace"])

        self.tree = {}
        for c in classes:
            class_name = make_namespace_class(c["namespace"], c["name"])
            inheritance = dict(clean_inheritance(c, self.namespace_by_class_name))
            self.tree[class_name] = inheritance

        self.n_template_point_types = {k: len(v) for inheritance in self.tree.values() for k, v in inheritance.items()}

    def get_class_namespace(self, class_name):
        assert "::" in class_name
        return class_name[class_name.rfind("::"):]

    def breadth_first_iterator(self, start_class=None):
        all_inheritances = [make_namespace_class(self.get_class_namespace(class_), i)
                            for class_, inheritance in self.tree.items()
                            for i in inheritance]
        if start_class is None:
            queue = deque(elem for elem in self.tree if elem not in all_inheritances)
        else:
            queue = deque([start_class])
        while queue:
            class_ = queue.popleft()
            yield class_
            for inherits in self.tree.get(class_, []):
                queue.append(inherits)

    def leaf_iterator(self):
        stack = list(sorted(self.tree.keys()))  # sort to output same result everytime
        seen = set()
        while stack:
            for class_name in stack:
                inheritance = set(self.tree.get(class_name).keys())
                inheritance = set((i[:i.find("<")] if "<" in i else i for i in inheritance))
                inheritance_current_namespace = set([make_namespace_class(class_name[:class_name.rfind("::")], i)
                                                 for i in inheritance])
                is_base_class = not inheritance
                inheritance_is_seen = not inheritance - seen or not (inheritance_current_namespace - seen)
                all_external = all(any(i.startswith(e) for e in EXTERNAL_INHERITANCE) for i in inheritance)
                if any([is_base_class, inheritance_is_seen, all_external]):
                    yield class_name
                    seen.add(class_name)
                    stack.remove(class_name)
                    break
            else:
                raise ValueError("Couldn't find base classes for: %s" % stack)

    def split_namespace_class(self, class_):
        return class_[:class_.rfind("::")], class_[class_.rfind("::") + 2:]

    def get_point_types_with_dependencies(self, classes_point_types):
        types = defaultdict(list)
        for class_ in self.breadth_first_iterator():
            namespace, class_name = self.split_namespace_class(class_)
            point_types_from_yml = classes_point_types.get(class_name)
            if point_types_from_yml:
                point_types = unpack_point_types(point_types_from_yml)
                n_point_types = len(point_types[0])
                types[class_name] += point_types
                for base_class_ in self.breadth_first_iterator(class_):
                    _, base_class_name = self.split_namespace_class(base_class_)
                    if base_class_name == class_name or base_class_name in IGNORE_INHERITED_INSTANTIATIONS:
                        continue
                    n_point_types_base = self.n_template_point_types[base_class_]
                    if n_point_types_base != n_point_types:
                        types_filter = INHERITED_TEMPLATED_TYPES_FILTER.get(base_class_)
                        if not types_filter:  # todo: clean this
                            types_filter = INHERITED_TEMPLATED_TYPES_FILTER.get(base_class_name)
                        if types_filter:
                            filtered = types_filter
                            types[base_class_name] += [tuple(t for i, t in enumerate(types_) if i in filtered) for
                                                       types_ in
                                                       point_types]
                        # BaseClass<PointInT> CurrentClass<PointInT, PointOutT>
                        # todo: Here I am hoping this inheritance is always ordered from the first point type
                        types[base_class_name] += [types_[:n_point_types_base] for types_ in point_types]
                    else:
                        types[base_class_name] += point_types
        return {class_name: list(sorted(set(v))) for class_name, v in types.items()}


def fix_templated_inheritance(inherits):
    while True:
        for n, i in enumerate(inherits[:]):
            if not parentheses_are_balanced(i, "<>"):
                inherits = inherits[:n] + ["%s, %s" % (i, inherits[n + 1])] + inherits[n + 2:]
                break
        else:
            break
    return inherits


def clean_inheritance(class_, namespace_by_class_name=None, replace_with_templated_typename=True):
    inheritance = [i["class"] for i in class_["inherits"]]
    inheritance = fix_templated_inheritance(inheritance)

    template = class_.get("template", "").replace("\n", "")
    template_typenames = {}
    if template:
        splitted = template[template.find("<") + 1: template.rfind(">") - 1].split("typename")
        for s in splitted:
            if "=" in s:
                val = s[s.find("=") + 1:].strip()
                template_typenames[s[:s.find("=")].strip()] = val

    for inherits in inheritance:
        template_types = tuple()
        if any(inherits.startswith(i) for i in SKIPPED_INHERITANCE):
            continue
        typename = template_typenames.get(inherits)
        if typename and replace_with_templated_typename:
            inherits = template_typenames.get(inherits, inherits)
        elif not typename:
            # deal with templates
            if "<" in inherits:
                template_types = tuple([s.strip() for s in inherits[inherits.find("<") + 1:-1].split(",")])
                inherits = inherits[:inherits.find("<")]

            namespace = class_["namespace"]
            is_external_inheritance = any(inherits.startswith(i) for i in EXTERNAL_INHERITANCE)
            if not is_external_inheritance:
                if namespace_by_class_name:
                    namespaces = namespace_by_class_name.get(inherits)
                    if namespaces:
                        namespace = namespaces[0]

                inherits = make_namespace_class(namespace, inherits)

        yield (inherits, template_types)
