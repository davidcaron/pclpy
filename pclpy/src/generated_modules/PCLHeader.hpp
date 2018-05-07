
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/PCLHeader.h>



void definePCLHeader(py::module &m) {
    using Class = pcl::PCLHeader;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PCLHeader");
    cls.def(py::init<>());
    cls.def_readwrite("seq", &Class::seq);
    cls.def_readwrite("stamp", &Class::stamp);
    cls.def_readwrite("frame_id", &Class::frame_id);
}

void definePCLHeaderFunctions1(py::module &m) {
    // Operators not implemented (operator==);
}

void definePCLHeaderFunctions(py::module &m) {
    definePCLHeaderFunctions1(m);
}

void definePCLHeaderClasses(py::module &sub_module) {
    definePCLHeader(sub_module);
    definePCLHeaderFunctions(sub_module);
}