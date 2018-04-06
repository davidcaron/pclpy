
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/orr_octree_zprojection.h>

using namespace pcl::recognition;


void defineRecognitionORROctreeZProjection(py::module &m) {
    using Class = recognition::ORROctreeZProjection;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "ORROctreeZProjection");
    cls.def(py::init<>());
    cls.def("build", py::overload_cast<const ORROctree &, float, float> (&Class::build));
    cls.def("clear", &Class::clear);
    cls.def("get_pixel", py::overload_cast<const float *> (&Class::getPixel, py::const_));
    cls.def("get_pixel", py::overload_cast<const float *> (&Class::getPixel));
    cls.def("get_pixel", py::overload_cast<int, int> (&Class::getPixel, py::const_));
}

void defineRecognitionOrrOctreeZprojectionClasses(py::module &sub_module) {
    defineRecognitionORROctreeZProjection(sub_module);
}