import inspect
import traceback

from pclpy import pcl


def test_initialize_everything():
    pass


def find_classes(obj=None):
    """
    Find all wrapped PCL classes
    """
    if obj is None:
        obj = pcl
    if inspect.ismodule(obj):
        classes = []
        elements = [getattr(obj, c) for c in dir(obj) if not c.startswith("__")]
        for element in elements:
            classes += find_classes(element)
        return classes
    elif inspect.isclass(obj) and hasattr(obj, "__init__"):
        return [obj]
    else:
        return []


def test_generate_initializations():
    """
    Helper function to look for binding errors
    """
    classes = find_classes()
    for class_ in classes:
        print(class_.__module__ + "." + class_.__name__)
        try:
            class_()
        except TypeError:
            exc = traceback.format_exc()
            if "incompatible constructor arguments" in exc:
                continue
            if "No constructor defined" in exc:
                continue
            print(class_.__name__)
            pass
