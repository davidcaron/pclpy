
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/Vertices.h>



void defineVertices(py::module &m) {
    using Class = pcl::Vertices;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Vertices");
    cls.def(py::init<>());
    cls.def_readonly("vertices", &Class::vertices);
}

void defineVerticesClasses(py::module &sub_module) {
    defineVertices(sub_module);
}