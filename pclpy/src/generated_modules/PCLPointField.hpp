
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/PCLPointField.h>



void definePCLPointField(py::module &m) {
    using Class = pcl::PCLPointField;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PCLPointField");
    py::enum_<Class::PointFieldTypes>(cls, "PointFieldTypes")
        .value("INT8", Class::PointFieldTypes::INT8)
        .value("UINT8", Class::PointFieldTypes::UINT8)
        .value("INT16", Class::PointFieldTypes::INT16)
        .value("UINT16", Class::PointFieldTypes::UINT16)
        .value("INT32", Class::PointFieldTypes::INT32)
        .value("UINT32", Class::PointFieldTypes::UINT32)
        .value("FLOAT32", Class::PointFieldTypes::FLOAT32)
        .value("FLOAT64", Class::PointFieldTypes::FLOAT64)
        .export_values();
    cls.def(py::init<>());
    cls.def_readwrite("name", &Class::name);
    cls.def_readwrite("offset", &Class::offset);
    cls.def_readwrite("datatype", &Class::datatype);
    cls.def_readwrite("count", &Class::count);
}

void definePCLPointFieldFunctions(py::module &m) {
}

void definePCLPointFieldClasses(py::module &sub_module) {
    definePCLPointField(sub_module);
    definePCLPointFieldFunctions(sub_module);
}