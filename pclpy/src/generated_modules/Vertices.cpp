
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include <pcl/Vertices.h>



void defineVertices(py::module &m) {
    using Class = pcl::Vertices;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Vertices");
    cls.def(py::init<>());
    cls.def_readwrite("vertices", &Class::vertices);
}

void defineVerticesFunctions(py::module &m) {
}

void defineVerticesClasses(py::module &sub_module) {
    defineVertices(sub_module);
    defineVerticesFunctions(sub_module);
}