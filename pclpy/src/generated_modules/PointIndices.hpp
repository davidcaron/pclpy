
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/PointIndices.h>



void definePointIndices(py::module &m) {
    using Class = PointIndices;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PointIndices");
    cls.def(py::init<>());
    cls.def_readonly("header", &Class::header);
    cls.def_readonly("indices", &Class::indices);
}

void definePointIndicesClasses(py::module &sub_module) {
    definePointIndices(sub_module);
}