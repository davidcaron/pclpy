
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/2d/edge.h>



template <typename PointInT, typename PointOutT>
void define2dEdge(py::module &m, std::string const & suffix) {
    using Class = Edge<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::OUTPUT_TYPE>(cls, "output_type")
        .value("OUTPUT_Y", Class::OUTPUT_TYPE::OUTPUT_Y)
        .value("OUTPUT_X", Class::OUTPUT_TYPE::OUTPUT_X)
        .value("OUTPUT_X_Y", Class::OUTPUT_TYPE::OUTPUT_X_Y)
        .value("OUTPUT_MAGNITUDE", Class::OUTPUT_TYPE::OUTPUT_MAGNITUDE)
        .value("OUTPUT_DIRECTION", Class::OUTPUT_TYPE::OUTPUT_DIRECTION)
        .value("OUTPUT_MAGNITUDE_DIRECTION", Class::OUTPUT_TYPE::OUTPUT_MAGNITUDE_DIRECTION)
        .value("OUTPUT_ALL", Class::OUTPUT_TYPE::OUTPUT_ALL)
        .export_values();
    py::enum_<Class::DETECTOR_KERNEL_TYPE>(cls, "detector_kernel_type")
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
    cls.def("set_output_type", &Class::setOutputType);
    cls.def("set_hysteresis_threshold_low", &Class::setHysteresisThresholdLow);
    cls.def("set_hysteresis_threshold_high", &Class::setHysteresisThresholdHigh);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("sobel_magnitude_direction", &Class::sobelMagnitudeDirection);
    cls.def("canny", &Class::canny);
    cls.def("detect_edge", &Class::detectEdge);
    cls.def("detect_edge_canny", &Class::detectEdgeCanny);
    cls.def("detect_edge_sobel", &Class::detectEdgeSobel);
    cls.def("detect_edge_prewitt", &Class::detectEdgePrewitt);
    cls.def("detect_edge_roberts", &Class::detectEdgeRoberts);
    cls.def("detect_edge_lo_g", &Class::detectEdgeLoG);
    cls.def("compute_derivative_x_central", &Class::computeDerivativeXCentral);
    cls.def("compute_derivative_y_central", &Class::computeDerivativeYCentral);
    cls.def("compute_derivative_x_forward", &Class::computeDerivativeXForward);
    cls.def("compute_derivative_y_forward", &Class::computeDerivativeYForward);
    cls.def("compute_derivative_x_backward", &Class::computeDerivativeXBackward);
    cls.def("compute_derivative_y_backward", &Class::computeDerivativeYBackward);
    cls.def("apply_filter", &Class::applyFilter);
        
}

void define2dEdgeClasses(py::module &sub_module) {
}