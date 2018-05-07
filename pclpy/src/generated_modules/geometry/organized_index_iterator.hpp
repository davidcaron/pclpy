
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/geometry/organized_index_iterator.h>



void defineGeometryOrganizedIndexIterator(py::module &m) {
    using Class = pcl::OrganizedIndexIterator;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "OrganizedIndexIterator");
    // Operators not implemented (operator++);
    // Operators not implemented (operator++);
    // Operators not implemented (operator*);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    cls.def("reset", &Class::reset);
    cls.def("getIndex", &Class::getIndex);
    cls.def("getRowIndex", &Class::getRowIndex);
    cls.def("getColumnIndex", &Class::getColumnIndex);
}

void defineGeometryOrganizedIndexIteratorFunctions(py::module &m) {
}

void defineGeometryOrganizedIndexIteratorClasses(py::module &sub_module) {
    defineGeometryOrganizedIndexIterator(sub_module);
    defineGeometryOrganizedIndexIteratorFunctions(sub_module);
}