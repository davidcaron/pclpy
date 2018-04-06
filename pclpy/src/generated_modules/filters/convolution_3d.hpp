
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/convolution_3d.h>

using namespace pcl::filters;


template<typename PointInT, typename PointOutT>
void defineFiltersConvolvingKernel(py::module &m, std::string const & suffix) {
    using Class = filters::ConvolvingKernel<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("set_input_cloud", &Class::setInputCloud);
    // Operators not implemented (operator());
    cls.def("init_compute", &Class::initCompute);
    cls.def("make_infinite", &Class::makeInfinite);
        
}

template<typename PointInT, typename PointOutT>
void defineFiltersGaussianKernel(py::module &m, std::string const & suffix) {
    using Class = filters::GaussianKernel<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, ConvolvingKernel<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_sigma", &Class::setSigma);
    cls.def("set_threshold_relative_to_sigma", &Class::setThresholdRelativeToSigma);
    cls.def("set_threshold", &Class::setThreshold);
    cls.def("init_compute", &Class::initCompute);
    // Operators not implemented (operator());
        
}

template<typename PointInT, typename PointOutT>
void defineFiltersGaussianKernelRGB(py::module &m, std::string const & suffix) {
    using Class = filters::GaussianKernelRGB<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, GaussianKernel<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    // Operators not implemented (operator());
        
}

template <typename PointIn, typename PointOut, typename KernelT>
void defineFiltersConvolution3D(py::module &m, std::string const & suffix) {
    using Class = filters::Convolution3D<PointIn, PointOut, KernelT>;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloudOut = Class::PointCloudOut;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PCLBase<PointIn>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
    cls.def("set_kernel", &Class::setKernel);
    cls.def_property("search_surface", &Class::getSearchSurface, &Class::setSearchSurface);
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def_property("radius_search", &Class::getRadiusSearch, &Class::setRadiusSearch);
    cls.def("convolve", py::overload_cast<PointCloudOut &> (&Class::convolve));
        
}

void defineFiltersConvolution3dClasses(py::module &sub_module) {
}