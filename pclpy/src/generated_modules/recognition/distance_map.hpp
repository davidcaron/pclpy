
#include <pcl/recognition/distance_map.h>



void defineRecognitionDistanceMap(py::module &m) {
    using Class = pcl::DistanceMap;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DistanceMap");
    cls.def(py::init<>());
    cls.def("resize", &Class::resize, "width"_a, "height"_a);
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
    cls.def("getData", py::overload_cast<> (&Class::getData));
}

void defineRecognitionDistanceMapFunctions(py::module &m) {
}

void defineRecognitionDistanceMapClasses(py::module &sub_module) {
    defineRecognitionDistanceMap(sub_module);
}