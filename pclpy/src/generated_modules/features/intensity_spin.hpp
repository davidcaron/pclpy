
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/intensity_spin.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesIntensitySpinEstimation(py::module &m, std::string const & suffix) {
    using Class = IntensitySpinEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("nr_distance_bins", &Class::getNrDistanceBins, &Class::setNrDistanceBins);
    cls.def_property("nr_intensity_bins", &Class::getNrIntensityBins, &Class::setNrIntensityBins);
    cls.def_property("smoothing_bandwith", &Class::getSmoothingBandwith, &Class::setSmoothingBandwith);
    cls.def_readonly("nr_distance_bins_", &Class::nr_distance_bins_);
    cls.def_readonly("nr_intensity_bins_", &Class::nr_intensity_bins_);
    cls.def_readonly("sigma_", &Class::sigma_);
    cls.def("compute_intensity_spin_image", &Class::computeIntensitySpinImage);
    cls.def("compute_feature", &Class::computeFeature);
        
}

void defineFeaturesIntensitySpinClasses(py::module &sub_module) {
    py::module sub_module_IntensitySpinEstimation = sub_module.def_submodule("IntensitySpinEstimation", "Submodule IntensitySpinEstimation");
    defineFeaturesIntensitySpinEstimation<PointXYZI, Histogram<20>>(sub_module_IntensitySpinEstimation, "PointXYZI_Histogram<20>");
}