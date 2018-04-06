
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/geometry/line_iterator.h>



void defineGeometryLineIterator(py::module &m) {
    using Class = LineIterator;
    py::class_<Class, OrganizedIndexIterator, boost::shared_ptr<Class>> cls(m, "LineIterator");
    py::enum_<Class::Neighborhood>(cls, "neighborhood")
        .value("Neighbor4", Class::Neighborhood::Neighbor4)
        .value("Neighbor8", Class::Neighborhood::Neighbor8)
        .export_values();
    cls.def(py::init<unsigned, unsigned, unsigned, unsigned, unsigned, Class::Neighborhood>(), "x_start"_a, "y_start"_a, "x_end"_a, "y_end"_a, "width"_a, "neighborhood"_a=Class::Neighbor8);
    // Operators not implemented (operator++);
    cls.def("is_valid", py::overload_cast<> (&Class::isValid, py::const_));
    cls.def("reset", &Class::reset);
}

void defineGeometryLineIteratorClasses(py::module &sub_module) {
    defineGeometryLineIterator(sub_module);
}