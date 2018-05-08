
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

#include <pcl/features/shot_omp.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT1344, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesSHOTColorEstimationOMP(py::module &m, std::string const & suffix) {
    using Class = pcl::SHOTColorEstimationOMP<PointInT, PointNT, PointOutT, PointRFT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::SHOTColorEstimation<PointInT, PointNT, PointOutT, PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT352, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesSHOTEstimationOMP(py::module &m, std::string const & suffix) {
    using Class = pcl::SHOTEstimationOMP<PointInT, PointNT, PointOutT, PointRFT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::SHOTEstimation<PointInT, PointNT, PointOutT, PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineFeaturesShotOmpFunctions(py::module &m) {
}

void defineFeaturesShotOmpClasses(py::module &sub_module) {
    py::module sub_module_SHOTColorEstimationOMP = sub_module.def_submodule("SHOTColorEstimationOMP", "Submodule SHOTColorEstimationOMP");
    defineFeaturesSHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>(sub_module_SHOTColorEstimationOMP, "PointXYZRGB_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTColorEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>(sub_module_SHOTColorEstimationOMP, "PointXYZRGBA_Normal_SHOT1344_ReferenceFrame");
    py::module sub_module_SHOTEstimationOMP = sub_module.def_submodule("SHOTEstimationOMP", "Submodule SHOTEstimationOMP");
    defineFeaturesSHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimationOMP, "PointXYZ_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimationOMP, "PointXYZI_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimationOMP, "PointXYZRGB_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimationOMP, "PointXYZRGBA_Normal_SHOT352_ReferenceFrame");
}