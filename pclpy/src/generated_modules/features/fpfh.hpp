
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/fpfh.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::FPFHSignature33>
void defineFeaturesFPFHEstimation(py::module &m, std::string const & suffix) {
    using Class = FPFHEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("nr_subdivisions", &Class::getNrSubdivisions, &Class::setNrSubdivisions);
    cls.def("compute_pair_features", &Class::computePairFeatures);
    cls.def("compute_point_spfh_signature", &Class::computePointSPFHSignature);
    cls.def("weight_point_spfh_signature", &Class::weightPointSPFHSignature);
        
}

void defineFeaturesFpfhClasses(py::module &sub_module) {
    py::module sub_module_FPFHEstimation = sub_module.def_submodule("FPFHEstimation", "Submodule FPFHEstimation");
    defineFeaturesFPFHEstimation<PointNormal, Normal, FPFHSignature33>(sub_module_FPFHEstimation, "PointNormal_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimation<PointNormal, PointNormal, FPFHSignature33>(sub_module_FPFHEstimation, "PointNormal_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimation<PointXYZ, Normal, FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZ_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimation<PointXYZ, PointNormal, FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZ_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimation<PointXYZI, Normal, FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZI_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimation<PointXYZI, PointNormal, FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZI_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimation<PointXYZRGB, Normal, FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZRGB_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimation<PointXYZRGB, PointNormal, FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZRGB_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimation<PointXYZRGBA, Normal, FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZRGBA_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimation<PointXYZRGBA, PointNormal, FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZRGBA_PointNormal_FPFHSignature33");
}