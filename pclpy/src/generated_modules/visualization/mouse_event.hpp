
#include <pcl/visualization/mouse_event.h>

using namespace pcl::visualization;


void defineVisualizationMouseEvent(py::module &m) {
    using Class = pcl::visualization::MouseEvent;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "MouseEvent");
    py::enum_<Class::Type>(cls, "Type")
        .value("MouseMove", Class::Type::MouseMove)
        .value("MouseButtonPress", Class::Type::MouseButtonPress)
        .value("MouseButtonRelease", Class::Type::MouseButtonRelease)
        .value("MouseScrollDown", Class::Type::MouseScrollDown)
        .value("MouseScrollUp", Class::Type::MouseScrollUp)
        .value("MouseDblClick", Class::Type::MouseDblClick)
        .export_values();
    py::enum_<Class::MouseButton>(cls, "MouseButton")
        .value("NoButton", Class::MouseButton::NoButton)
        .value("LeftButton", Class::MouseButton::LeftButton)
        .value("MiddleButton", Class::MouseButton::MiddleButton)
        .value("RightButton", Class::MouseButton::RightButton)
        .value("VScroll", Class::MouseButton::VScroll)
        .export_values();
    cls.def(py::init<Class::Type, Class::MouseButton, unsigned int, unsigned int, bool, bool, bool, bool>(), "type"_a, "button"_a, "x"_a, "y"_a, "alt"_a, "ctrl"_a, "shift"_a, "selection_mode"_a=false);
    cls.def("setType", &Class::setType, "type"_a);
    cls.def("setButton", &Class::setButton, "button"_a);
    cls.def("getType", &Class::getType);
    cls.def("getButton", &Class::getButton);
    cls.def("getX", &Class::getX);
    cls.def("getY", &Class::getY);
    cls.def("getKeyboardModifiers", &Class::getKeyboardModifiers);
    cls.def("getSelectionMode", &Class::getSelectionMode);
}

void defineVisualizationMouseEventFunctions(py::module &m) {
}

void defineVisualizationMouseEventClasses(py::module &sub_module) {
    defineVisualizationMouseEvent(sub_module);
}