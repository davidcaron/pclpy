
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/surface/ear_clipping.h>



void defineSurfaceEarClipping(py::module &m) {
    using Class = pcl::EarClipping;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::MeshProcessing, boost::shared_ptr<Class>> cls(m, "EarClipping");
    cls.def(py::init<>());
}

void defineSurfaceEarClippingFunctions(py::module &m) {
}

void defineSurfaceEarClippingClasses(py::module &sub_module) {
    defineSurfaceEarClipping(sub_module);
}