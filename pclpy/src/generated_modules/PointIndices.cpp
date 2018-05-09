
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include <pcl/PointIndices.h>



void definePointIndices(py::module &m) {
    using Class = pcl::PointIndices;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PointIndices");
    cls.def(py::init<>());
    cls.def_readwrite("header", &Class::header);
    cls.def_readwrite("indices", &Class::indices);
}

void definePointIndicesFunctions(py::module &m) {
}

void definePointIndicesClasses(py::module &sub_module) {
    definePointIndices(sub_module);
    definePointIndicesFunctions(sub_module);
}