
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/convolution.h>

using namespace pcl::filters;


template <typename PointIn, typename PointOut>
void defineFiltersConvolution(py::module &m, std::string const & suffix) {
    using Class = filters::Convolution<PointIn, PointOut>;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::BORDERS_POLICY>(cls, "borders_policy")
        .value("BORDERS_POLICY_IGNORE", Class::BORDERS_POLICY::BORDERS_POLICY_IGNORE)
        .value("BORDERS_POLICY_MIRROR", Class::BORDERS_POLICY::BORDERS_POLICY_MIRROR)
        .value("BORDERS_POLICY_DUPLICATE", Class::BORDERS_POLICY::BORDERS_POLICY_DUPLICATE)
        .export_values();
    cls.def(py::init<>());
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("set_kernel", &Class::setKernel);
    cls.def_property("borders_policy", &Class::getBordersPolicy, &Class::setBordersPolicy);
    cls.def_property("distance_threshold", &Class::getDistanceThreshold, &Class::setDistanceThreshold);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
    cls.def("convolve", py::overload_cast<const Eigen::ArrayXf &, const Eigen::ArrayXf &, PointCloudOut &> (&Class::convolve));
    cls.def("convolve", py::overload_cast<PointCloudOut &> (&Class::convolve));
    cls.def("convolve_rows", &Class::convolveRows);
    cls.def("convolve_cols", &Class::convolveCols);
        
}

void defineFiltersConvolutionClasses(py::module &sub_module) {
}