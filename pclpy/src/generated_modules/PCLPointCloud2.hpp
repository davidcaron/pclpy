
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/PCLPointCloud2.h>



void definePCLPointCloud2(py::module &m) {
    using Class = pcl::PCLPointCloud2;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PCLPointCloud2");
    cls.def(py::init<>());
    cls.def_readwrite("header", &Class::header);
    cls.def_readwrite("height", &Class::height);
    cls.def_readwrite("width", &Class::width);
    cls.def_readwrite("fields", &Class::fields);
    cls.def_readwrite("is_bigendian", &Class::is_bigendian);
    cls.def_readwrite("point_step", &Class::point_step);
    cls.def_readwrite("row_step", &Class::row_step);
    cls.def_readwrite("data", &Class::data);
    cls.def_readwrite("is_dense", &Class::is_dense);
}

void definePCLPointCloud2Functions(py::module &m) {
}

void definePCLPointCloud2Classes(py::module &sub_module) {
    definePCLPointCloud2(sub_module);
    definePCLPointCloud2Functions(sub_module);
}