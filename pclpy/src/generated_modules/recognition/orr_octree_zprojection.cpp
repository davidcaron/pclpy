
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/recognition/ransac_based/orr_octree_zprojection.h>

using namespace pcl::recognition;


void defineRecognitionORROctreeZProjection(py::module &m) {
    using Class = pcl::recognition::ORROctreeZProjection;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "ORROctreeZProjection");
    cls.def(py::init<>());
    cls.def("build", py::overload_cast<const ORROctree &, float, float> (&Class::build), "input"_a, "eps_front"_a, "eps_back"_a);
    cls.def("clear", &Class::clear);
    cls.def("getPixelCoordinates", &Class::getPixelCoordinates, "p"_a, "x"_a, "y"_a);
    cls.def("getPixel", py::overload_cast<const float *> (&Class::getPixel, py::const_), "p"_a);
    cls.def("getPixel", py::overload_cast<const float *> (&Class::getPixel), "p"_a);
    cls.def("getFullPixels", &Class::getFullPixels);
    cls.def("getPixel", py::overload_cast<int, int> (&Class::getPixel, py::const_), "i"_a, "j"_a);
    cls.def("getPixelSize", &Class::getPixelSize);
    cls.def("getBounds", py::overload_cast<> (&Class::getBounds, py::const_));
    cls.def("getNumberOfPixels", &Class::getNumberOfPixels, "num_x"_a, "num_y"_a);
}

void defineRecognitionOrrOctreeZprojectionFunctions(py::module &m) {
}

void defineRecognitionOrrOctreeZprojectionClasses(py::module &sub_module) {
    defineRecognitionORROctreeZProjection(sub_module);
}