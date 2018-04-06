
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/shot.h>



template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesSHOTEstimationBase(py::module &m, std::string const & suffix) {
    using Class = SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, FeatureWithLocalReferenceFrames<PointInT,PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("lrf_radius", &Class::getLRFRadius, &Class::setLRFRadius);
    cls.def("compute_point_shot", &Class::computePointSHOT);
        
}

template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT1344, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesSHOTColorEstimation(py::module &m, std::string const & suffix) {
    using Class = SHOTColorEstimation<PointInT, PointNT, PointOutT, PointRFT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, SHOTEstimationBase<PointInT,PointNT,PointOutT,PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool, bool>(), "describe_shape"_a=true, "describe_color"_a=true);
    cls.def("compute_point_shot", &Class::computePointSHOT);
    cls.def("rgb2_cielab", &Class::RGB2CIELAB);
        
}

template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT352, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesSHOTEstimation(py::module &m, std::string const & suffix) {
    using Class = SHOTEstimation<PointInT, PointNT, PointOutT, PointRFT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, SHOTEstimationBase<PointInT,PointNT,PointOutT,PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute_point_shot", &Class::computePointSHOT);
        
}

void defineFeaturesShotClasses(py::module &sub_module) {
    py::module sub_module_SHOTEstimationBase = sub_module.def_submodule("SHOTEstimationBase", "Submodule SHOTEstimationBase");
    defineFeaturesSHOTEstimationBase<PointXYZ, Normal, SHOT1344, ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZ_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<PointXYZ, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZ_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<PointXYZI, Normal, SHOT1344, ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZI_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<PointXYZI, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZI_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<PointXYZRGB, Normal, SHOT1344, ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZRGB_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<PointXYZRGB, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZRGB_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<PointXYZRGBA, Normal, SHOT1344, ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZRGBA_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTEstimationBase<PointXYZRGBA, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimationBase, "PointXYZRGBA_Normal_SHOT352_ReferenceFrame");
    py::module sub_module_SHOTColorEstimation = sub_module.def_submodule("SHOTColorEstimation", "Submodule SHOTColorEstimation");
    defineFeaturesSHOTColorEstimation<PointXYZRGB, Normal, SHOT1344, ReferenceFrame>(sub_module_SHOTColorEstimation, "PointXYZRGB_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTColorEstimation<PointXYZRGBA, Normal, SHOT1344, ReferenceFrame>(sub_module_SHOTColorEstimation, "PointXYZRGBA_Normal_SHOT1344_ReferenceFrame");
    py::module sub_module_SHOTEstimation = sub_module.def_submodule("SHOTEstimation", "Submodule SHOTEstimation");
    defineFeaturesSHOTEstimation<PointXYZ, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimation, "PointXYZ_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimation<PointXYZI, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimation, "PointXYZI_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimation<PointXYZRGB, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimation, "PointXYZRGB_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimation<PointXYZRGBA, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimation, "PointXYZRGBA_Normal_SHOT352_ReferenceFrame");
}