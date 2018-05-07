
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/filters/convolution.h>

using namespace pcl::filters;


template <typename PointIn, typename PointOut>
void defineFiltersConvolution(py::module &m, std::string const & suffix) {
    using Class = pcl::filters::Convolution<PointIn, PointOut>;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::BORDERS_POLICY>(cls, "BORDERS_POLICY")
        .value("BORDERS_POLICY_IGNORE", Class::BORDERS_POLICY::BORDERS_POLICY_IGNORE)
        .value("BORDERS_POLICY_MIRROR", Class::BORDERS_POLICY::BORDERS_POLICY_MIRROR)
        .value("BORDERS_POLICY_DUPLICATE", Class::BORDERS_POLICY::BORDERS_POLICY_DUPLICATE)
        .export_values();
    cls.def(py::init<>());
    cls.def("convolveRows", &Class::convolveRows, "output"_a);
    cls.def("convolveCols", &Class::convolveCols, "output"_a);
    cls.def("convolve", py::overload_cast<const Eigen::ArrayXf &, const Eigen::ArrayXf &, PointCloudOut &> (&Class::convolve), "h_kernel"_a, "v_kernel"_a, "output"_a);
    cls.def("convolve", py::overload_cast<PointCloudOut &> (&Class::convolve), "output"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setKernel", &Class::setKernel, "kernel"_a);
    cls.def("setBordersPolicy", &Class::setBordersPolicy, "policy"_a);
    cls.def("setDistanceThreshold", &Class::setDistanceThreshold, "threshold"_a);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
    cls.def("getBordersPolicy", &Class::getBordersPolicy);
    cls.def("getDistanceThreshold", &Class::getDistanceThreshold);
        
}

void defineFiltersConvolutionFunctions(py::module &m) {
}

void defineFiltersConvolutionClasses(py::module &sub_module) {
}