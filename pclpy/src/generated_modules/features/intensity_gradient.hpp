
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/intensity_gradient.h>



template <typename PointInT, typename PointNT, typename PointOutT, typename IntensitySelectorT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineFeaturesIntensityGradientEstimation(py::module &m, std::string const & suffix) {
    using Class = IntensityGradientEstimation<PointInT, PointNT, PointOutT, IntensitySelectorT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineFeaturesIntensityGradientClasses(py::module &sub_module) {
    py::module sub_module_IntensityGradientEstimation = sub_module.def_submodule("IntensityGradientEstimation", "Submodule IntensityGradientEstimation");
    defineFeaturesIntensityGradientEstimation<PointXYZI, Normal, IntensityGradient>(sub_module_IntensityGradientEstimation, "PointXYZI_Normal_IntensityGradient");
}