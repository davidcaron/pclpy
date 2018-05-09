
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/shot.h>



template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesSHOTEstimationBase(py::module &m, std::string const & suffix) {
    using Class = pcl::SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, pcl::FeatureWithLocalReferenceFrames<PointInT, PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("computePointSHOT", &Class::computePointSHOT, "index"_a, "indices"_a, "sqr_dists"_a, "shot"_a);
    cls.def("setLRFRadius", &Class::setLRFRadius, "radius"_a);
    cls.def("getLRFRadius", &Class::getLRFRadius);
        
}

template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT1344, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesSHOTColorEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::SHOTColorEstimation<PointInT, PointNT, PointOutT, PointRFT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool, bool>(), "describe_shape"_a=true, "describe_color"_a=true);
    cls.def("computePointSHOT", &Class::computePointSHOT, "index"_a, "indices"_a, "sqr_dists"_a, "shot"_a);
    cls.def_static("RGB2CIELAB", &Class::RGB2CIELAB, "R"_a, "G"_a, "B"_a, "L"_a, "A"_a, "B2"_a);
        
}

template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT352, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesSHOTEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::SHOTEstimation<PointInT, PointNT, PointOutT, PointRFT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computePointSHOT", &Class::computePointSHOT, "index"_a, "indices"_a, "sqr_dists"_a, "shot"_a);
        
}

void defineFeaturesShotFunctions(py::module &m) {
}

void defineFeaturesShotClasses(py::module &sub_module) {
    py::module sub_module_SHOTEstimationBase = sub_module.def_submodule("SHOTEstimationBase", "Submodule SHOTEstimationBase");
    defineFeaturesSHOTEstimationBase<pcl::PointXYZ, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZ_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<pcl::PointXYZ, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZ_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<pcl::PointXYZI, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZI_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<pcl::PointXYZI, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZI_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZRGB_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZRGB_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZRGBA_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZRGBA_Normal_SHOT352_ReferenceFrame");
    py::module sub_module_SHOTColorEstimation = sub_module.def_submodule("SHOTColorEstimation", "Submodule SHOTColorEstimation");
    defineFeaturesSHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>(sub_module_SHOTColorEstimation, "PointXYZRGB_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTColorEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>(sub_module_SHOTColorEstimation, "PointXYZRGBA_Normal_SHOT1344_ReferenceFrame");
    py::module sub_module_SHOTEstimation = sub_module.def_submodule("SHOTEstimation", "Submodule SHOTEstimation");
    defineFeaturesSHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimation, "PointXYZ_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimation<pcl::PointXYZI, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimation, "PointXYZI_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimation, "PointXYZRGB_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame>(sub_module_SHOTEstimation, "PointXYZRGBA_Normal_SHOT352_ReferenceFrame");
}