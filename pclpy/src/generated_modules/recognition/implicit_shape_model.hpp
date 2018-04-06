
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/implicit_shape_model.h>



void defineRecognitionISMPeak(py::module &m) {
    using Class = ISMPeak;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "ISMPeak");
    cls.def_readonly("density", &Class::density);
    cls.def_readonly("class_id", &Class::class_id);
}

void defineRecognitionImplicitShapeModelClasses(py::module &sub_module) {
    defineRecognitionISMPeak(sub_module);
}