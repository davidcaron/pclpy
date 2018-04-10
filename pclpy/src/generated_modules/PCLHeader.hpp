
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
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
    cls.def_readonly("seq", &Class::seq);
    cls.def_readonly("stamp", &Class::stamp);
    cls.def_readonly("frame_id", &Class::frame_id);
}

void definePCLHeaderClasses(py::module &sub_module) {
    definePCLHeader(sub_module);
}