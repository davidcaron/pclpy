
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/2d/kernel.h>



template<typename PointT>
void define2dkernel(py::module &m, std::string const & suffix) {
    using Class = kernel<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::KERNEL_ENUM>(cls, "kernel_enum")
        .value("SOBEL_X", Class::KERNEL_ENUM::SOBEL_X)
        .value("SOBEL_Y", Class::KERNEL_ENUM::SOBEL_Y)
        .value("PREWITT_X", Class::KERNEL_ENUM::PREWITT_X)
        .value("PREWITT_Y", Class::KERNEL_ENUM::PREWITT_Y)
        .value("ROBERTS_X", Class::KERNEL_ENUM::ROBERTS_X)
        .value("ROBERTS_Y", Class::KERNEL_ENUM::ROBERTS_Y)
        .value("LOG", Class::KERNEL_ENUM::LOG)
        .value("DERIVATIVE_CENTRAL_X", Class::KERNEL_ENUM::DERIVATIVE_CENTRAL_X)
        .value("DERIVATIVE_FORWARD_X", Class::KERNEL_ENUM::DERIVATIVE_FORWARD_X)
        .value("DERIVATIVE_BACKWARD_X", Class::KERNEL_ENUM::DERIVATIVE_BACKWARD_X)
        .value("DERIVATIVE_CENTRAL_Y", Class::KERNEL_ENUM::DERIVATIVE_CENTRAL_Y)
        .value("DERIVATIVE_FORWARD_Y", Class::KERNEL_ENUM::DERIVATIVE_FORWARD_Y)
        .value("DERIVATIVE_BACKWARD_Y", Class::KERNEL_ENUM::DERIVATIVE_BACKWARD_Y)
        .value("GAUSSIAN", Class::KERNEL_ENUM::GAUSSIAN)
        .export_values();
    cls.def(py::init<>());
    cls.def("set_kernel_type", &Class::setKernelType);
    cls.def("set_kernel_size", &Class::setKernelSize);
    cls.def("set_kernel_sigma", &Class::setKernelSigma);
    cls.def_readonly("kernel_size_", &Class::kernel_size_);
    cls.def_readonly("sigma_", &Class::sigma_);
    cls.def_readonly("kernel_type_", &Class::kernel_type_);
    cls.def("fetch_kernel", &Class::fetchKernel);
    cls.def("gaussian_kernel", &Class::gaussianKernel);
    cls.def("lo_g_kernel", &Class::loGKernel);
    cls.def("sobel_kernel_x", &Class::sobelKernelX);
    cls.def("prewitt_kernel_x", &Class::prewittKernelX);
    cls.def("roberts_kernel_x", &Class::robertsKernelX);
    cls.def("sobel_kernel_y", &Class::sobelKernelY);
    cls.def("prewitt_kernel_y", &Class::prewittKernelY);
    cls.def("roberts_kernel_y", &Class::robertsKernelY);
    cls.def("derivative_x_central_kernel", &Class::derivativeXCentralKernel);
    cls.def("derivative_y_central_kernel", &Class::derivativeYCentralKernel);
    cls.def("derivative_x_forward_kernel", &Class::derivativeXForwardKernel);
    cls.def("derivative_y_forward_kernel", &Class::derivativeYForwardKernel);
    cls.def("derivative_x_backward_kernel", &Class::derivativeXBackwardKernel);
    cls.def("derivative_y_backward_kernel", &Class::derivativeYBackwardKernel);
        
}

void define2dKernelClasses(py::module &sub_module) {
}