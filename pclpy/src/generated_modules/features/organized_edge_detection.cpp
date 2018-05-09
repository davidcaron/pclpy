
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/organized_edge_detection.h>



template <typename PointT, typename PointLT>
void defineFeaturesOrganizedEdgeBase(py::module &m, std::string const & suffix) {
    using Class = pcl::OrganizedEdgeBase<PointT, PointLT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_readonly_static("num_of_edgetype_", &Class::num_of_edgetype_);
    cls.def("compute", &Class::compute, "labels"_a, "label_indices"_a);
    cls.def("setDepthDisconThreshold", &Class::setDepthDisconThreshold, "th"_a);
    cls.def("setMaxSearchNeighbors", &Class::setMaxSearchNeighbors, "max_dist"_a);
    cls.def("setEdgeType", &Class::setEdgeType, "edge_types"_a);
    cls.def("getDepthDisconThreshold", &Class::getDepthDisconThreshold);
    cls.def("getMaxSearchNeighbors", &Class::getMaxSearchNeighbors);
    cls.def("getEdgeType", &Class::getEdgeType);
        
}

template <typename PointT, typename PointNT, typename PointLT>
void defineFeaturesOrganizedEdgeFromNormals(py::module &m, std::string const & suffix) {
    using Class = pcl::OrganizedEdgeFromNormals<PointT, PointNT, PointLT>;
    py::class_<Class, pcl::OrganizedEdgeBase<PointT, PointLT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute", &Class::compute, "labels"_a, "label_indices"_a);
    cls.def("setInputNormals", &Class::setInputNormals, "normals"_a);
    cls.def("setHCCannyLowThreshold", &Class::setHCCannyLowThreshold, "th"_a);
    cls.def("setHCCannyHighThreshold", &Class::setHCCannyHighThreshold, "th"_a);
    cls.def("getInputNormals", &Class::getInputNormals);
    cls.def("getHCCannyLowThreshold", &Class::getHCCannyLowThreshold);
    cls.def("getHCCannyHighThreshold", &Class::getHCCannyHighThreshold);
        
}

template <typename PointT, typename PointLT>
void defineFeaturesOrganizedEdgeFromRGB(py::module &m, std::string const & suffix) {
    using Class = pcl::OrganizedEdgeFromRGB<PointT, PointLT>;
    py::class_<Class, pcl::OrganizedEdgeBase<PointT, PointLT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute", &Class::compute, "labels"_a, "label_indices"_a);
    cls.def("setRGBCannyLowThreshold", &Class::setRGBCannyLowThreshold, "th"_a);
    cls.def("setRGBCannyHighThreshold", &Class::setRGBCannyHighThreshold, "th"_a);
    cls.def("getRGBCannyLowThreshold", &Class::getRGBCannyLowThreshold);
    cls.def("getRGBCannyHighThreshold", &Class::getRGBCannyHighThreshold);
        
}

template <typename PointT, typename PointNT, typename PointLT>
void defineFeaturesOrganizedEdgeFromRGBNormals(py::module &m, std::string const & suffix) {
    using Class = pcl::OrganizedEdgeFromRGBNormals<PointT, PointNT, PointLT>;
    py::class_<Class, pcl::OrganizedEdgeFromRGB<PointT, PointLT>, pcl::OrganizedEdgeFromNormals<PointT, PointNT, PointLT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute", &Class::compute, "labels"_a, "label_indices"_a);
        
}

void defineFeaturesOrganizedEdgeDetectionFunctions(py::module &m) {
}

void defineFeaturesOrganizedEdgeDetectionClasses(py::module &sub_module) {
    py::module sub_module_OrganizedEdgeBase = sub_module.def_submodule("OrganizedEdgeBase", "Submodule OrganizedEdgeBase");
    defineFeaturesOrganizedEdgeBase<pcl::InterestPoint, pcl::Label>(sub_module_OrganizedEdgeBase, "InterestPoint_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointDEM, pcl::Label>(sub_module_OrganizedEdgeBase, "PointDEM_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeBase, "PointNormal_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeBase, "PointSurfel_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointWithRange, pcl::Label>(sub_module_OrganizedEdgeBase, "PointWithRange_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointWithScale, pcl::Label>(sub_module_OrganizedEdgeBase, "PointWithScale_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointWithViewpoint, pcl::Label>(sub_module_OrganizedEdgeBase, "PointWithViewpoint_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointXYZ, pcl::Label>(sub_module_OrganizedEdgeBase, "PointXYZ_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointXYZHSV, pcl::Label>(sub_module_OrganizedEdgeBase, "PointXYZHSV_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointXYZI, pcl::Label>(sub_module_OrganizedEdgeBase, "PointXYZI_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeBase, "PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointXYZL, pcl::Label>(sub_module_OrganizedEdgeBase, "PointXYZL_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeBase, "PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>(sub_module_OrganizedEdgeBase, "PointXYZRGB_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>(sub_module_OrganizedEdgeBase, "PointXYZRGBA_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointXYZRGBL, pcl::Label>(sub_module_OrganizedEdgeBase, "PointXYZRGBL_Label");
    defineFeaturesOrganizedEdgeBase<pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeBase, "PointXYZRGBNormal_Label");
    py::module sub_module_OrganizedEdgeFromNormals = sub_module.def_submodule("OrganizedEdgeFromNormals", "Submodule OrganizedEdgeFromNormals");
    defineFeaturesOrganizedEdgeFromNormals<pcl::InterestPoint, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::InterestPoint, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::InterestPoint, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::InterestPoint, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::InterestPoint, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::InterestPoint, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointDEM, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointDEM, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointDEM, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointDEM, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointDEM, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointDEM, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointNormal, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointNormal, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointNormal, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointNormal, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointNormal, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointNormal, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointSurfel, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointSurfel, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointSurfel, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointSurfel, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointSurfel, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointSurfel, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithRange, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithRange, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithRange, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithRange, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithRange, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithRange, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithScale, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithScale, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithScale, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithScale, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithScale, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithScale, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithViewpoint, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithViewpoint, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithViewpoint, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithViewpoint, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithViewpoint, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointWithViewpoint, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZ, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZ, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZ, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZ, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZ, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZHSV, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZHSV, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZHSV, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZHSV, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZHSV, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZHSV, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZI, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZI, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZI, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZI, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZI, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZI, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZINormal, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZINormal, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZINormal, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZINormal, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZINormal, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZL, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZL, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZL, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZL, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZL, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZL, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZLNormal, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZLNormal, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZLNormal, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZLNormal, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZLNormal, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZLNormal, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGB, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGB, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGB, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGB, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBA, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBA, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBA, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBA, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBL, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBL, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBL, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBL, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBL, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBL, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBNormal, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBNormal, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBNormal, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_PointXYZRGBNormal_Label");
    py::module sub_module_OrganizedEdgeFromRGB = sub_module.def_submodule("OrganizedEdgeFromRGB", "Submodule OrganizedEdgeFromRGB");
    defineFeaturesOrganizedEdgeFromRGB<pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromRGB, "PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGB<pcl::PointXYZRGB, pcl::Label>(sub_module_OrganizedEdgeFromRGB, "PointXYZRGB_Label");
    defineFeaturesOrganizedEdgeFromRGB<pcl::PointXYZRGBA, pcl::Label>(sub_module_OrganizedEdgeFromRGB, "PointXYZRGBA_Label");
    defineFeaturesOrganizedEdgeFromRGB<pcl::PointXYZRGBL, pcl::Label>(sub_module_OrganizedEdgeFromRGB, "PointXYZRGBL_Label");
    defineFeaturesOrganizedEdgeFromRGB<pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGB, "PointXYZRGBNormal_Label");
    py::module sub_module_OrganizedEdgeFromRGBNormals = sub_module.def_submodule("OrganizedEdgeFromRGBNormals", "Submodule OrganizedEdgeFromRGBNormals");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointSurfel, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_Normal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointSurfel, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointSurfel, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointSurfel, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointSurfel, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointSurfel, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGB, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_Normal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGB, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGB, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGB, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGB, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_Normal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBL, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_Normal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBL, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBL, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBL, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBL, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBL, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBNormal, pcl::Normal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_Normal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBNormal, pcl::PointNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBNormal, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBNormal, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBNormal, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_PointXYZRGBNormal_Label");
}