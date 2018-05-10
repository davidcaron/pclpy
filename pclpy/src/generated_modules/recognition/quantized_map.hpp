
#include <pcl/recognition/quantized_map.h>



void defineRecognitionQuantizedMap(py::module &m) {
    using Class = pcl::QuantizedMap;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "QuantizedMap");
    cls.def(py::init<>());
    cls.def(py::init<size_t, size_t>(), "width"_a, "height"_a);
    cls.def_readwrite("data_", &Class::data_);
    cls.def_readwrite("width_", &Class::width_);
    cls.def_readwrite("height_", &Class::height_);
    cls.def("resize", &Class::resize, "width"_a, "height"_a);
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def_static("spreadQuantizedMap", &Class::spreadQuantizedMap, "input_map"_a, "output_map"_a, "spreading_size"_a);
    cls.def("serialize", &Class::serialize, "stream"_a);
    cls.def("deserialize", &Class::deserialize, "stream"_a);
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
    cls.def("getData", py::overload_cast<> (&Class::getData));
    cls.def("getData", py::overload_cast<> (&Class::getData, py::const_));
    cls.def("getSubMap", &Class::getSubMap, "x"_a, "y"_a, "width"_a, "height"_a);
}

void defineRecognitionQuantizedMapFunctions(py::module &m) {
}

void defineRecognitionQuantizedMapClasses(py::module &sub_module) {
    defineRecognitionQuantizedMap(sub_module);
}