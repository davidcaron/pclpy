
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/correspondence.h>



void defineCorrespondence(py::module &m) {
    using Class = pcl::Correspondence;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Correspondence");
    cls.def(py::init<>());
    cls.def(py::init<int, int, float>(), "_index_query"_a, "_index_match"_a, "_distance"_a);
    cls.def_readonly("index_query", &Class::index_query);
    cls.def_readonly("index_match", &Class::index_match);
    cls.def_readonly("distance", &Class::distance);
    cls.def_readonly("weight", &Class::weight);
}

void definePointCorrespondence3D(py::module &m) {
    using Class = pcl::PointCorrespondence3D;
    py::class_<Class, Correspondence, boost::shared_ptr<Class>> cls(m, "PointCorrespondence3D");
    cls.def(py::init<>());
    cls.def_readonly("point1", &Class::point1);
    cls.def_readonly("point2", &Class::point2);
}

void definePointCorrespondence6D(py::module &m) {
    using Class = pcl::PointCorrespondence6D;
    py::class_<Class, PointCorrespondence3D, boost::shared_ptr<Class>> cls(m, "PointCorrespondence6D");
    cls.def_readonly("transformation", &Class::transformation);
}

void defineCorrespondenceClasses(py::module &sub_module) {
    defineCorrespondence(sub_module);
    definePointCorrespondence3D(sub_module);
    definePointCorrespondence6D(sub_module);
}