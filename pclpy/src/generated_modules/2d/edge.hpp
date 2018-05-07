
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/2d/edge.h>



template <typename PointInT, typename PointOutT>
void define2dEdge(py::module &m, std::string const & suffix) {
    using Class = pcl::Edge<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::OUTPUT_TYPE>(cls, "OUTPUT_TYPE")
        .value("OUTPUT_Y", Class::OUTPUT_TYPE::OUTPUT_Y)
        .value("OUTPUT_X", Class::OUTPUT_TYPE::OUTPUT_X)
        .value("OUTPUT_X_Y", Class::OUTPUT_TYPE::OUTPUT_X_Y)
        .value("OUTPUT_MAGNITUDE", Class::OUTPUT_TYPE::OUTPUT_MAGNITUDE)
        .value("OUTPUT_DIRECTION", Class::OUTPUT_TYPE::OUTPUT_DIRECTION)
        .value("OUTPUT_MAGNITUDE_DIRECTION", Class::OUTPUT_TYPE::OUTPUT_MAGNITUDE_DIRECTION)
        .value("OUTPUT_ALL", Class::OUTPUT_TYPE::OUTPUT_ALL)
        .export_values();
    py::enum_<Class::DETECTOR_KERNEL_TYPE>(cls, "DETECTOR_KERNEL_TYPE")
        .value("CANNY", Class::DETECTOR_KERNEL_TYPE::CANNY)
        .value("SOBEL", Class::DETECTOR_KERNEL_TYPE::SOBEL)
        .value("PREWITT", Class::DETECTOR_KERNEL_TYPE::PREWITT)
        .value("ROBERTS", Class::DETECTOR_KERNEL_TYPE::ROBERTS)
        .value("LOG", Class::DETECTOR_KERNEL_TYPE::LOG)
        .value("DERIVATIVE_CENTRAL", Class::DETECTOR_KERNEL_TYPE::DERIVATIVE_CENTRAL)
        .value("DERIVATIVE_FORWARD", Class::DETECTOR_KERNEL_TYPE::DERIVATIVE_FORWARD)
        .value("DERIVATIVE_BACKWARD", Class::DETECTOR_KERNEL_TYPE::DERIVATIVE_BACKWARD)
        .export_values();
    cls.def(py::init<>());
    cls.def("sobelMagnitudeDirection", &Class::sobelMagnitudeDirection, "input_x"_a, "input_y"_a, "output"_a);
    cls.def("canny", &Class::canny, "input_x"_a, "input_y"_a, "output"_a);
    cls.def("detectEdge", &Class::detectEdge, "output"_a);
    cls.def("detectEdgeCanny", &Class::detectEdgeCanny, "output"_a);
    cls.def("detectEdgeSobel", &Class::detectEdgeSobel, "output"_a);
    cls.def("detectEdgePrewitt", &Class::detectEdgePrewitt, "output"_a);
    cls.def("detectEdgeRoberts", &Class::detectEdgeRoberts, "output"_a);
    cls.def("detectEdgeLoG", &Class::detectEdgeLoG, "kernel_sigma"_a, "kernel_size"_a, "output"_a);
    cls.def("computeDerivativeXCentral", &Class::computeDerivativeXCentral, "output"_a);
    cls.def("computeDerivativeYCentral", &Class::computeDerivativeYCentral, "output"_a);
    cls.def("computeDerivativeXForward", &Class::computeDerivativeXForward, "output"_a);
    cls.def("computeDerivativeYForward", &Class::computeDerivativeYForward, "output"_a);
    cls.def("computeDerivativeXBackward", &Class::computeDerivativeXBackward, "output"_a);
    cls.def("computeDerivativeYBackward", &Class::computeDerivativeYBackward, "output"_a);
    cls.def("applyFilter", &Class::applyFilter, "&"_a);
    cls.def("setOutputType", &Class::setOutputType, "output_type"_a);
    cls.def("setHysteresisThresholdLow", &Class::setHysteresisThresholdLow, "threshold"_a);
    cls.def("setHysteresisThresholdHigh", &Class::setHysteresisThresholdHigh, "threshold"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "input"_a);
        
}

void define2dEdgeFunctions(py::module &m) {
}

void define2dEdgeClasses(py::module &sub_module) {
}