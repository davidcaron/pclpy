
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

#pragma warning(disable : 4267)
#include <pcl/recognition/implicit_shape_model.h>



void defineRecognitionISMPeak(py::module &m) {
    using Class = pcl::ISMPeak;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "ISMPeak");
    cls.def_readwrite("density", &Class::density);
    cls.def_readwrite("class_id", &Class::class_id);
}

void defineRecognitionImplicitShapeModelFunctions(py::module &m) {
}

void defineRecognitionImplicitShapeModelClasses(py::module &sub_module) {
    defineRecognitionISMPeak(sub_module);
}