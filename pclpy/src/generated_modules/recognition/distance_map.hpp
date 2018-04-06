
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/distance_map.h>



void defineRecognitionDistanceMap(py::module &m) {
    using Class = DistanceMap;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DistanceMap");
    cls.def(py::init<>());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("resize", &Class::resize);
}

void defineRecognitionDistanceMapClasses(py::module &sub_module) {
    defineRecognitionDistanceMap(sub_module);
}