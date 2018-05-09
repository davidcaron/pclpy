
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/2d/convolution.h>



template <typename PointT>
void define2dConvolution(py::module &m, std::string const & suffix) {
    using Class = pcl::Convolution<PointT>;
    py::class_<Class, pcl::Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::BOUNDARY_OPTIONS_ENUM>(cls, "BOUNDARY_OPTIONS_ENUM")
        .value("BOUNDARY_OPTION_CLAMP", Class::BOUNDARY_OPTIONS_ENUM::BOUNDARY_OPTION_CLAMP)
        .value("BOUNDARY_OPTION_MIRROR", Class::BOUNDARY_OPTIONS_ENUM::BOUNDARY_OPTION_MIRROR)
        .value("BOUNDARY_OPTION_ZERO_PADDING", Class::BOUNDARY_OPTIONS_ENUM::BOUNDARY_OPTION_ZERO_PADDING)
        .export_values();
    cls.def(py::init<>());
    cls.def("filter", &Class::filter, "output"_a);
    cls.def("setKernel", &Class::setKernel, "kernel"_a);
    cls.def("setBoundaryOptions", &Class::setBoundaryOptions, "boundary_options"_a);
        
}

void define2dPointXYZIEdge(py::module &m) {
    using Class = pcl::PointXYZIEdge;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PointXYZIEdge");
    cls.def_readwrite("magnitude", &Class::magnitude);
    cls.def_readwrite("direction", &Class::direction);
    cls.def_readwrite("magnitude_x", &Class::magnitude_x);
    cls.def_readwrite("magnitude_y", &Class::magnitude_y);
}

void define2dConvolutionFunctions(py::module &m) {
}

void define2dConvolutionClasses(py::module &sub_module) {
    define2dPointXYZIEdge(sub_module);
}