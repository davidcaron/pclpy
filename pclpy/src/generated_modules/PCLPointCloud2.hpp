
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/PCLPointCloud2.h>



void definePCLPointCloud2(py::module &m) {
    using Class = PCLPointCloud2;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PCLPointCloud2");
    cls.def(py::init<>());
    cls.def_readonly("header", &Class::header);
    cls.def_readonly("height", &Class::height);
    cls.def_readonly("width", &Class::width);
    cls.def_readonly("fields", &Class::fields);
    cls.def_readonly("is_bigendian", &Class::is_bigendian);
    cls.def_readonly("point_step", &Class::point_step);
    cls.def_readonly("row_step", &Class::row_step);
    cls.def_readonly("data", &Class::data);
    cls.def_readonly("is_dense", &Class::is_dense);
}

void definePCLPointCloud2Classes(py::module &sub_module) {
    definePCLPointCloud2(sub_module);
}