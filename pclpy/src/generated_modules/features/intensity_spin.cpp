
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

#include <pcl/features/intensity_spin.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesIntensitySpinEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::IntensitySpinEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_readwrite("nr_distance_bins_", &Class::nr_distance_bins_);
    cls.def_readwrite("nr_intensity_bins_", &Class::nr_intensity_bins_);
    cls.def_readwrite("sigma_", &Class::sigma_);
    cls.def("computeIntensitySpinImage", &Class::computeIntensitySpinImage, "cloud"_a, "radius"_a, "sigma"_a, "k"_a, "indices"_a, "squared_distances"_a, "intensity_spin_image"_a);
    cls.def("computeFeature", &Class::computeFeature, "output"_a);
    cls.def("setNrDistanceBins", &Class::setNrDistanceBins, "nr_distance_bins"_a);
    cls.def("setNrIntensityBins", &Class::setNrIntensityBins, "nr_intensity_bins"_a);
    cls.def("setSmoothingBandwith", &Class::setSmoothingBandwith, "sigma"_a);
    cls.def("getNrDistanceBins", &Class::getNrDistanceBins);
    cls.def("getNrIntensityBins", &Class::getNrIntensityBins);
    cls.def("getSmoothingBandwith", &Class::getSmoothingBandwith);
        
}

void defineFeaturesIntensitySpinFunctions(py::module &m) {
}

void defineFeaturesIntensitySpinClasses(py::module &sub_module) {
    py::module sub_module_IntensitySpinEstimation = sub_module.def_submodule("IntensitySpinEstimation", "Submodule IntensitySpinEstimation");
    defineFeaturesIntensitySpinEstimation<pcl::PointXYZI, pcl::Histogram<20>>(sub_module_IntensitySpinEstimation, "PointXYZI_Histogram<20>");
}