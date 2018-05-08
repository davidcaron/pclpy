
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/filters/convolution_3d.h>

using namespace pcl::filters;


template <typename PointIn, typename PointOut, typename KernelT>
void defineFiltersConvolution3D(py::module &m, std::string const & suffix) {
    using Class = pcl::filters::Convolution3D<PointIn, PointOut, KernelT>;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloudOut = Class::PointCloudOut;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::PCLBase<pcl::filters::PointIn>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("convolve", py::overload_cast<PointCloudOut &> (&Class::convolve), "output"_a);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
    cls.def("setKernel", &Class::setKernel, "kernel"_a);
    cls.def("setSearchSurface", &Class::setSearchSurface, "cloud"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("setRadiusSearch", &Class::setRadiusSearch, "radius"_a);
    cls.def("getSearchSurface", &Class::getSearchSurface);
    cls.def("getSearchMethod", &Class::getSearchMethod);
    cls.def("getRadiusSearch", &Class::getRadiusSearch);
        
}

template<typename PointInT, typename PointOutT>
void defineFiltersConvolvingKernel(py::module &m, std::string const & suffix) {
    using Class = pcl::filters::ConvolvingKernel<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    // Operators not implemented (operator());
    cls.def("initCompute", &Class::initCompute);
    cls.def_static("makeInfinite", &Class::makeInfinite, "p"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "input"_a);
        
}

template<typename PointInT, typename PointOutT>
void defineFiltersGaussianKernel(py::module &m, std::string const & suffix) {
    using Class = pcl::filters::GaussianKernel<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::filters::ConvolvingKernel<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("initCompute", &Class::initCompute);
    // Operators not implemented (operator());
    cls.def("setSigma", &Class::setSigma, "sigma"_a);
    cls.def("setThresholdRelativeToSigma", &Class::setThresholdRelativeToSigma, "sigma_coefficient"_a);
    cls.def("setThreshold", &Class::setThreshold, "threshold"_a);
        
}

template<typename PointInT, typename PointOutT>
void defineFiltersGaussianKernelRGB(py::module &m, std::string const & suffix) {
    using Class = pcl::filters::GaussianKernelRGB<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::filters::GaussianKernel<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    // Operators not implemented (operator());
        
}

void defineFiltersConvolution3dFunctions(py::module &m) {
}

void defineFiltersConvolution3dClasses(py::module &sub_module) {
}