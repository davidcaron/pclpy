
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/ear_clipping.h>



void defineSurfaceEarClipping(py::module &m) {
    using Class = EarClipping;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, MeshProcessing, boost::shared_ptr<Class>> cls(m, "EarClipping");
    cls.def(py::init<>());
}

void defineSurfaceEarClippingClasses(py::module &sub_module) {
    defineSurfaceEarClipping(sub_module);
}