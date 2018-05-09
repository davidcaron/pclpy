
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/intensity_gradient.h>



template <typename PointInT, typename PointNT, typename PointOutT, typename IntensitySelectorT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineFeaturesIntensityGradientEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::IntensityGradientEstimation<PointInT, PointNT, PointOutT, IntensitySelectorT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineFeaturesIntensityGradientFunctions(py::module &m) {
}

void defineFeaturesIntensityGradientClasses(py::module &sub_module) {
    py::module sub_module_IntensityGradientEstimation = sub_module.def_submodule("IntensityGradientEstimation", "Submodule IntensityGradientEstimation");
    defineFeaturesIntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient>(sub_module_IntensityGradientEstimation, "PointXYZI_Normal_IntensityGradient");
}