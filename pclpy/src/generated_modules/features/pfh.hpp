
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/pfh.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::PFHSignature125>
void defineFeaturesPFHEstimation(py::module &m, std::string const & suffix) {
    using Class = PFHEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("maximum_cache_size", &Class::getMaximumCacheSize, &Class::setMaximumCacheSize);
    cls.def_property("use_internal_cache", &Class::getUseInternalCache, &Class::setUseInternalCache);
    cls.def("compute_pair_features", &Class::computePairFeatures);
    cls.def("compute_point_pfh_signature", &Class::computePointPFHSignature);
        
}

void defineFeaturesPfhClasses(py::module &sub_module) {
    py::module sub_module_PFHEstimation = sub_module.def_submodule("PFHEstimation", "Submodule PFHEstimation");
    defineFeaturesPFHEstimation<PointXYZ, Normal, PFHSignature125>(sub_module_PFHEstimation, "PointXYZ_Normal_PFHSignature125");
    defineFeaturesPFHEstimation<PointXYZI, Normal, PFHSignature125>(sub_module_PFHEstimation, "PointXYZI_Normal_PFHSignature125");
    defineFeaturesPFHEstimation<PointXYZRGB, Normal, PFHSignature125>(sub_module_PFHEstimation, "PointXYZRGB_Normal_PFHSignature125");
    defineFeaturesPFHEstimation<PointXYZRGBA, Normal, PFHSignature125>(sub_module_PFHEstimation, "PointXYZRGBA_Normal_PFHSignature125");
}