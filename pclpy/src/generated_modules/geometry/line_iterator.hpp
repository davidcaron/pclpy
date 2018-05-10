
#include <pcl/geometry/line_iterator.h>



void defineGeometryLineIterator(py::module &m) {
    using Class = pcl::LineIterator;
    py::class_<Class, pcl::OrganizedIndexIterator, boost::shared_ptr<Class>> cls(m, "LineIterator");
    py::enum_<Class::Neighborhood>(cls, "Neighborhood")
        .value("Neighbor4", Class::Neighborhood::Neighbor4)
        .value("Neighbor8", Class::Neighborhood::Neighbor8)
        .export_values();
    cls.def(py::init<unsigned, unsigned, unsigned, unsigned, unsigned, Class::Neighborhood>(), "x_start"_a, "y_start"_a, "x_end"_a, "y_end"_a, "width"_a, "neighborhood"_a=Class::Neighbor8);
    // Operators not implemented (operator++);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    cls.def("reset", &Class::reset);
    cls.def("getRowIndex", &Class::getRowIndex);
    cls.def("getColumnIndex", &Class::getColumnIndex);
}

void defineGeometryLineIteratorFunctions(py::module &m) {
}

void defineGeometryLineIteratorClasses(py::module &sub_module) {
    defineGeometryLineIterator(sub_module);
    defineGeometryLineIteratorFunctions(sub_module);
}