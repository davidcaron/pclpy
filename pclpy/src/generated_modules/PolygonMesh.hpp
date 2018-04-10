
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/PolygonMesh.h>



void definePolygonMesh(py::module &m) {
    using Class = pcl::PolygonMesh;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PolygonMesh");
    cls.def(py::init<>());
    cls.def_readonly("header", &Class::header);
    cls.def_readonly("cloud", &Class::cloud);
    cls.def_readonly("polygons", &Class::polygons);
}

void definePolygonMeshClasses(py::module &sub_module) {
    definePolygonMesh(sub_module);
}