
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/vfh.h>



template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
void defineFeaturesVFHEstimation(py::module &m, std::string const & suffix) {
    using Class = VFHEstimation<PointInT, PointNT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("view_point", &Class::getViewPoint, &Class::setViewPoint);
    cls.def("set_use_given_normal", &Class::setUseGivenNormal);
    cls.def("set_normal_to_use", &Class::setNormalToUse);
    cls.def("set_use_given_centroid", &Class::setUseGivenCentroid);
    cls.def("set_centroid_to_use", &Class::setCentroidToUse);
    cls.def("set_normalize_bins", &Class::setNormalizeBins);
    cls.def("set_normalize_distance", &Class::setNormalizeDistance);
    cls.def("set_fill_size_component", &Class::setFillSizeComponent);
    cls.def("compute_point_spfh_signature", &Class::computePointSPFHSignature);
    cls.def("compute", &Class::compute);
        
}

void defineFeaturesVfhClasses(py::module &sub_module) {
    py::module sub_module_VFHEstimation = sub_module.def_submodule("VFHEstimation", "Submodule VFHEstimation");
    defineFeaturesVFHEstimation<PointNormal, Normal, VFHSignature308>(sub_module_VFHEstimation, "PointNormal_Normal_VFHSignature308");
    defineFeaturesVFHEstimation<PointNormal, PointNormal, VFHSignature308>(sub_module_VFHEstimation, "PointNormal_PointNormal_VFHSignature308");
    defineFeaturesVFHEstimation<PointXYZ, Normal, VFHSignature308>(sub_module_VFHEstimation, "PointXYZ_Normal_VFHSignature308");
    defineFeaturesVFHEstimation<PointXYZ, PointNormal, VFHSignature308>(sub_module_VFHEstimation, "PointXYZ_PointNormal_VFHSignature308");
    defineFeaturesVFHEstimation<PointXYZI, Normal, VFHSignature308>(sub_module_VFHEstimation, "PointXYZI_Normal_VFHSignature308");
    defineFeaturesVFHEstimation<PointXYZI, PointNormal, VFHSignature308>(sub_module_VFHEstimation, "PointXYZI_PointNormal_VFHSignature308");
    defineFeaturesVFHEstimation<PointXYZRGB, Normal, VFHSignature308>(sub_module_VFHEstimation, "PointXYZRGB_Normal_VFHSignature308");
    defineFeaturesVFHEstimation<PointXYZRGB, PointNormal, VFHSignature308>(sub_module_VFHEstimation, "PointXYZRGB_PointNormal_VFHSignature308");
    defineFeaturesVFHEstimation<PointXYZRGBA, Normal, VFHSignature308>(sub_module_VFHEstimation, "PointXYZRGBA_Normal_VFHSignature308");
    defineFeaturesVFHEstimation<PointXYZRGBA, PointNormal, VFHSignature308>(sub_module_VFHEstimation, "PointXYZRGBA_PointNormal_VFHSignature308");
}