
#include <pcl/recognition/mask_map.h>



void defineRecognitionMaskMap(py::module &m) {
    using Class = pcl::MaskMap;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "MaskMap");
    cls.def(py::init<>());
    cls.def(py::init<size_t, size_t>(), "width"_a, "height"_a);
    cls.def("resize", &Class::resize, "width"_a, "height"_a);
    cls.def("unset", &Class::unset, "x"_a, "y"_a);
    cls.def("isSet", &Class::isSet, "x"_a, "y"_a);
    cls.def("reset", &Class::reset);
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("erode", &Class::erode, "eroded_mask"_a);
    cls.def("set", &Class::set, "x"_a, "y"_a);
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
    cls.def("getData", py::overload_cast<> (&Class::getData));
    cls.def("getData", py::overload_cast<> (&Class::getData, py::const_));
    cls.def_static("getDifferenceMask", &Class::getDifferenceMask, "mask0"_a, "mask1"_a, "diff_mask"_a);
}

void defineRecognitionMaskMapFunctions(py::module &m) {
}

void defineRecognitionMaskMapClasses(py::module &sub_module) {
    defineRecognitionMaskMap(sub_module);
}