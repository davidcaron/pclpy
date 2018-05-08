
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

#include <pcl/correspondence.h>



void defineCorrespondence(py::module &m) {
    using Class = pcl::Correspondence;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Correspondence");
    cls.def(py::init<>());
    cls.def(py::init<int, int, float>(), "_index_query"_a, "_index_match"_a, "_distance"_a);
    cls.def_readwrite("index_query", &Class::index_query);
    cls.def_readwrite("index_match", &Class::index_match);
    cls.def_readwrite("distance", &Class::distance);
    cls.def_readwrite("weight", &Class::weight);
}

void definePointCorrespondence3D(py::module &m) {
    using Class = pcl::PointCorrespondence3D;
    py::class_<Class, pcl::Correspondence, boost::shared_ptr<Class>> cls(m, "PointCorrespondence3D");
    cls.def(py::init<>());
    cls.def_readwrite("point1", &Class::point1);
    cls.def_readwrite("point2", &Class::point2);
}

void definePointCorrespondence6D(py::module &m) {
    using Class = pcl::PointCorrespondence6D;
    py::class_<Class, pcl::PointCorrespondence3D, boost::shared_ptr<Class>> cls(m, "PointCorrespondence6D");
    cls.def_readwrite("transformation", &Class::transformation);
}

void defineCorrespondenceFunctions1(py::module &m) {
    m.def("isBetterCorrespondence", py::overload_cast<const pcl::Correspondence &, const pcl::Correspondence &> (&pcl::isBetterCorrespondence), "pc1"_a, "pc2"_a);
}

void defineCorrespondenceFunctions(py::module &m) {
    defineCorrespondenceFunctions1(m);
}

void defineCorrespondenceClasses(py::module &sub_module) {
    defineCorrespondence(sub_module);
    definePointCorrespondence3D(sub_module);
    definePointCorrespondence6D(sub_module);
    defineCorrespondenceFunctions(sub_module);
}