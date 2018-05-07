
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/2d/kernel.h>



template<typename PointT>
void define2dkernel(py::module &m, std::string const & suffix) {
    using Class = pcl::kernel<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::KERNEL_ENUM>(cls, "KERNEL_ENUM")
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
    cls.def_readwrite("kernel_size_", &Class::kernel_size_);
    cls.def_readwrite("sigma_", &Class::sigma_);
    cls.def_readwrite("kernel_type_", &Class::kernel_type_);
    cls.def("fetchKernel", &Class::fetchKernel, "kernel"_a);
    cls.def("gaussianKernel", &Class::gaussianKernel, "kernel"_a);
    cls.def("loGKernel", &Class::loGKernel, "kernel"_a);
    cls.def("sobelKernelX", &Class::sobelKernelX, "kernel"_a);
    cls.def("prewittKernelX", &Class::prewittKernelX, "kernel"_a);
    cls.def("robertsKernelX", &Class::robertsKernelX, "kernel"_a);
    cls.def("sobelKernelY", &Class::sobelKernelY, "kernel"_a);
    cls.def("prewittKernelY", &Class::prewittKernelY, "kernel"_a);
    cls.def("robertsKernelY", &Class::robertsKernelY, "kernel"_a);
    cls.def("derivativeXCentralKernel", &Class::derivativeXCentralKernel, "kernel"_a);
    cls.def("derivativeYCentralKernel", &Class::derivativeYCentralKernel, "kernel"_a);
    cls.def("derivativeXForwardKernel", &Class::derivativeXForwardKernel, "kernel"_a);
    cls.def("derivativeYForwardKernel", &Class::derivativeYForwardKernel, "kernel"_a);
    cls.def("derivativeXBackwardKernel", &Class::derivativeXBackwardKernel, "kernel"_a);
    cls.def("derivativeYBackwardKernel", &Class::derivativeYBackwardKernel, "kernel"_a);
    cls.def("setKernelType", &Class::setKernelType, "kernel_type"_a);
    cls.def("setKernelSize", &Class::setKernelSize, "kernel_size"_a);
    cls.def("setKernelSigma", &Class::setKernelSigma, "kernel_sigma"_a);
        
}

void define2dKernelFunctions(py::module &m) {
}

void define2dKernelClasses(py::module &sub_module) {
}