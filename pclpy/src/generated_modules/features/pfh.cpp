
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

#include <pcl/features/pfh.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::PFHSignature125>
void defineFeaturesPFHEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::PFHEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computePairFeatures", &Class::computePairFeatures, "cloud"_a, "normals"_a, "p_idx"_a, "q_idx"_a, "f1"_a, "f2"_a, "f3"_a, "f4"_a);
    cls.def("computePointPFHSignature", &Class::computePointPFHSignature, "cloud"_a, "normals"_a, "indices"_a, "nr_split"_a, "pfh_histogram"_a);
    cls.def("setMaximumCacheSize", &Class::setMaximumCacheSize, "cache_size"_a);
    cls.def("setUseInternalCache", &Class::setUseInternalCache, "use_cache"_a);
    cls.def("getMaximumCacheSize", &Class::getMaximumCacheSize);
    cls.def("getUseInternalCache", &Class::getUseInternalCache);
        
}

void defineFeaturesPfhFunctions(py::module &m) {
}

void defineFeaturesPfhClasses(py::module &sub_module) {
    py::module sub_module_PFHEstimation = sub_module.def_submodule("PFHEstimation", "Submodule PFHEstimation");
    defineFeaturesPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125>(sub_module_PFHEstimation, "PointXYZ_Normal_PFHSignature125");
    defineFeaturesPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::PFHSignature125>(sub_module_PFHEstimation, "PointXYZI_Normal_PFHSignature125");
    defineFeaturesPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125>(sub_module_PFHEstimation, "PointXYZRGB_Normal_PFHSignature125");
    defineFeaturesPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHSignature125>(sub_module_PFHEstimation, "PointXYZRGBA_Normal_PFHSignature125");
}