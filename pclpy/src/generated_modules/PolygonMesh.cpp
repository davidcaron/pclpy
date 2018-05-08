
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include <pcl/PolygonMesh.h>



void definePolygonMesh(py::module &m) {
    using Class = pcl::PolygonMesh;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PolygonMesh");
    cls.def(py::init<>());
    cls.def_readwrite("header", &Class::header);
    cls.def_readwrite("cloud", &Class::cloud);
    cls.def_readwrite("polygons", &Class::polygons);
}

void definePolygonMeshFunctions(py::module &m) {
}

void definePolygonMeshClasses(py::module &sub_module) {
    definePolygonMesh(sub_module);
    definePolygonMeshFunctions(sub_module);
}