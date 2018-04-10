
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/PCLImage.h>



void definePCLImage(py::module &m) {
    using Class = pcl::PCLImage;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PCLImage");
    cls.def(py::init<>());
    cls.def_readonly("header", &Class::header);
    cls.def_readonly("height", &Class::height);
    cls.def_readonly("width", &Class::width);
    cls.def_readonly("encoding", &Class::encoding);
    cls.def_readonly("is_bigendian", &Class::is_bigendian);
    cls.def_readonly("step", &Class::step);
    cls.def_readonly("data", &Class::data);
}

void definePCLImageClasses(py::module &sub_module) {
    definePCLImage(sub_module);
}