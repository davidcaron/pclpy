
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/fpfh_omp.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesFPFHEstimationOMP(py::module &m, std::string const & suffix) {
    using Class = FPFHEstimationOMP<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, FPFHEstimation<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "nr_threads"_a=0);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
    cls.def_readonly("nr_bins_f1_", &Class::nr_bins_f1_);
    cls.def_readonly("nr_bins_f2_", &Class::nr_bins_f2_);
    cls.def_readonly("nr_bins_f3_", &Class::nr_bins_f3_);
        
}

void defineFeaturesFpfhOmpClasses(py::module &sub_module) {
    py::module sub_module_FPFHEstimationOMP = sub_module.def_submodule("FPFHEstimationOMP", "Submodule FPFHEstimationOMP");
    defineFeaturesFPFHEstimationOMP<PointNormal, Normal, FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointNormal_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<PointNormal, PointNormal, FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointNormal_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZ_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<PointXYZ, PointNormal, FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZ_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<PointXYZI, Normal, FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZI_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<PointXYZI, PointNormal, FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZI_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<PointXYZRGB, Normal, FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZRGB_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<PointXYZRGB, PointNormal, FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZRGB_PointNormal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<PointXYZRGBA, Normal, FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZRGBA_Normal_FPFHSignature33");
    defineFeaturesFPFHEstimationOMP<PointXYZRGBA, PointNormal, FPFHSignature33>(sub_module_FPFHEstimationOMP, "PointXYZRGBA_PointNormal_FPFHSignature33");
}