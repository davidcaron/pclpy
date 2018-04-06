
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/geometry/organized_index_iterator.h>



void defineGeometryOrganizedIndexIterator(py::module &m) {
    using Class = OrganizedIndexIterator;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "OrganizedIndexIterator");
    // Operators not implemented (operator++);
    // Operators not implemented (operator++);
    cls.def("is_valid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator*);
    cls.def("reset", &Class::reset);
}

void defineGeometryOrganizedIndexIteratorClasses(py::module &sub_module) {
    defineGeometryOrganizedIndexIterator(sub_module);
}