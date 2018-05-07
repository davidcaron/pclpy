
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/2d/morphology.h>



template <typename PointT>
void define2dMorphology(py::module &m, std::string const & suffix) {
    using Class = pcl::Morphology<PointT>;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::MORPHOLOGICAL_OPERATOR_TYPE>(cls, "MORPHOLOGICAL_OPERATOR_TYPE")
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
    cls.def_readwrite("operator_type", &Class::operator_type);
    cls.def("openingBinary", &Class::openingBinary, "output"_a);
    cls.def("closingBinary", &Class::closingBinary, "output"_a);
    cls.def("erosionBinary", &Class::erosionBinary, "output"_a);
    cls.def("dilationBinary", &Class::dilationBinary, "output"_a);
    cls.def("openingGray", &Class::openingGray, "output"_a);
    cls.def("closingGray", &Class::closingGray, "output"_a);
    cls.def("erosionGray", &Class::erosionGray, "output"_a);
    cls.def("dilationGray", &Class::dilationGray, "output"_a);
    cls.def("subtractionBinary", &Class::subtractionBinary, "output"_a, "input1"_a, "input2"_a);
    cls.def("unionBinary", &Class::unionBinary, "output"_a, "input1"_a, "input2"_a);
    cls.def("intersectionBinary", &Class::intersectionBinary, "output"_a, "input1"_a, "input2"_a);
    cls.def("structuringElementCircular", &Class::structuringElementCircular, "kernel"_a, "radius"_a);
    cls.def("structuringElementRectangle", &Class::structuringElementRectangle, "kernel"_a, "height"_a, "width"_a);
    cls.def("applyMorphologicalOperation", &Class::applyMorphologicalOperation, "output"_a);
    cls.def("setStructuringElement", &Class::setStructuringElement, "structuring_element"_a);
        
}

void define2dMorphologyFunctions(py::module &m) {
}

void define2dMorphologyClasses(py::module &sub_module) {
}