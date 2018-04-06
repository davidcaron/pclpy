
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/2d/morphology.h>



template <typename PointT>
void define2dMorphology(py::module &m, std::string const & suffix) {
    using Class = Morphology<PointT>;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::MORPHOLOGICAL_OPERATOR_TYPE>(cls, "morphological_operator_type")
        .value("EROSION_GRAY", Class::MORPHOLOGICAL_OPERATOR_TYPE::EROSION_GRAY)
        .value("DILATION_GRAY", Class::MORPHOLOGICAL_OPERATOR_TYPE::DILATION_GRAY)
        .value("OPENING_GRAY", Class::MORPHOLOGICAL_OPERATOR_TYPE::OPENING_GRAY)
        .value("CLOSING_GRAY", Class::MORPHOLOGICAL_OPERATOR_TYPE::CLOSING_GRAY)
        .value("EROSION_BINARY", Class::MORPHOLOGICAL_OPERATOR_TYPE::EROSION_BINARY)
        .value("DILATION_BINARY", Class::MORPHOLOGICAL_OPERATOR_TYPE::DILATION_BINARY)
        .value("OPENING_BINARY", Class::MORPHOLOGICAL_OPERATOR_TYPE::OPENING_BINARY)
        .value("CLOSING_BINARY", Class::MORPHOLOGICAL_OPERATOR_TYPE::CLOSING_BINARY)
        .export_values();
    cls.def(py::init<>());
    cls.def("set_structuring_element", &Class::setStructuringElement);
    cls.def_readonly("operator_type", &Class::operator_type);
    cls.def("opening_binary", &Class::openingBinary);
    cls.def("closing_binary", &Class::closingBinary);
    cls.def("erosion_binary", &Class::erosionBinary);
    cls.def("dilation_binary", &Class::dilationBinary);
    cls.def("opening_gray", &Class::openingGray);
    cls.def("closing_gray", &Class::closingGray);
    cls.def("erosion_gray", &Class::erosionGray);
    cls.def("dilation_gray", &Class::dilationGray);
    cls.def("subtraction_binary", &Class::subtractionBinary);
    cls.def("union_binary", &Class::unionBinary);
    cls.def("intersection_binary", &Class::intersectionBinary);
    cls.def("structuring_element_circular", &Class::structuringElementCircular);
    cls.def("structuring_element_rectangle", &Class::structuringElementRectangle);
    cls.def("apply_morphological_operation", &Class::applyMorphologicalOperation);
        
}

void define2dMorphologyClasses(py::module &sub_module) {
}