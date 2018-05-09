
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/fpfh_omp.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesFPFHEstimationOMP(py::module &m, std::string const & suffix) {
    using Class = pcl::FPFHEstimationOMP<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::FPFHEstimation<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "nr_threads"_a=0);
    cls.def_readwrite("nr_bins_f1_", &Class::nr_bins_f1_);
    cls.def_readwrite("nr_bins_f2_", &Class::nr_bins_f2_);
    cls.def_readwrite("nr_bins_f3_", &Class::nr_bins_f3_);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineFeaturesFpfhOmpFunctions(py::module &m) {
}

void defineFeaturesFpfhOmpClasses(py::module &sub_module) {
    py::module sub_module_FPFHEstimationOMP = sub_module.def_submodule("FPFHEstimationOMP", "Submodule FPFHEstimationOMP");
    defineFeaturesFPFHEstimationOMP<pcl::PointNormal, pcl::Normal, pcl::FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointNormal_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointNormal_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZ_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZ_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZI_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<pcl::PointXYZI, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZI_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZRGB_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZRGB_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZRGBA_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<pcl::PointXYZRGBA, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZRGBA_PointNormal_FPFHSignature33");
}