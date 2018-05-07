from collections import defaultdict, deque
from typing import List

from CppHeaderParser import CppClass

from generators.config import EXTERNAL_INHERITANCE, IGNORE_INHERITED_INSTANTIATIONS, INHERITED_TEMPLATED_TYPES_FILTER
from generators.point_types_utils import clean_inheritance, get_class_namespace
from generators.utils import make_namespace_class


class DependencyTree:
    def __init__(self, classes: List[CppClass]):
        """

        :param classes:
        """
        self.namespace_by_class_name = defaultdict(list)
        for c in classes:
            self.namespace_by_class_name[c["name"]].append(c["namespace"])

        self.tree = {}
        for c in classes:
            class_name = make_namespace_class(c["namespace"], c["name"])
            inheritance = dict(clean_inheritance(c, self.namespace_by_class_name))
            self.tree[class_name] = inheritance

        self.n_template_point_types = {k: len(v) for inheritance in self.tree.values() for k, v in inheritance.items()}

    def breadth_first_iterator(self, start_class=None):
        all_inheritances = [make_namespace_class(get_class_namespace(class_), i)
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
            point_types = classes_point_types.get(class_name)
            if point_types:
                n_point_types = len(point_types[0])
                types[class_name] += point_types
                for base_class_ in self.breadth_first_iterator(class_):
                    _, base_class_name = self.split_namespace_class(base_class_)
                    if base_class_name == class_name or base_class_name in IGNORE_INHERITED_INSTANTIATIONS:
                        continue
                    n_point_types_base = self.n_template_point_types[base_class_]
                    if n_point_types_base != n_point_types:
                        template_filter = INHERITED_TEMPLATED_TYPES_FILTER
                        types_filter = template_filter.get(base_class_, template_filter.get(base_class_name))
                        if types_filter:
                            for types_ in point_types:
                                filtered = tuple(t for i, t in enumerate(types_) if i in types_filter)
                                types[base_class_name].append(filtered)
                        types[base_class_name] += [types_[:n_point_types_base] for types_ in point_types]
                    else:
                        types[base_class_name] += point_types
        return {class_name: list(sorted(set(v))) for class_name, v in types.items()}
