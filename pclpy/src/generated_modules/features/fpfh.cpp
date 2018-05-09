
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/fpfh.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::FPFHSignature33>
void defineFeaturesFPFHEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::FPFHEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computePairFeatures", &Class::computePairFeatures, "cloud"_a, "normals"_a, "p_idx"_a, "q_idx"_a, "f1"_a, "f2"_a, "f3"_a, "f4"_a);
    cls.def("computePointSPFHSignature", &Class::computePointSPFHSignature, "cloud"_a, "normals"_a, "p_idx"_a, "row"_a, "indices"_a, "hist_f1"_a, "hist_f2"_a, "hist_f3"_a);
    cls.def("weightPointSPFHSignature", &Class::weightPointSPFHSignature, "hist_f1"_a, "hist_f2"_a, "hist_f3"_a, "indices"_a, "dists"_a, "fpfh_histogram"_a);
    cls.def("setNrSubdivisions", &Class::setNrSubdivisions, "nr_bins_f1"_a, "nr_bins_f2"_a, "nr_bins_f3"_a);
    cls.def("getNrSubdivisions", &Class::getNrSubdivisions, "nr_bins_f1"_a, "nr_bins_f2"_a, "nr_bins_f3"_a);
        
}

void defineFeaturesFpfhFunctions(py::module &m) {
}

void defineFeaturesFpfhClasses(py::module &sub_module) {
    py::module sub_module_FPFHEstimation = sub_module.def_submodule("FPFHEstimation", "Submodule FPFHEstimation");
    defineFeaturesFPFHEstimation<pcl::PointNormal, pcl::Normal, pcl::FPFHSignature33>(sub_module_FPFHEstimation, "PointNormal_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FPFHEstimation, "PointNormal_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZ_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZ_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZI_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZI_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZRGB_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZRGB_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZRGBA_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FPFHEstimation, "PointXYZRGBA_PointNormal_FPFHSignature33");
}