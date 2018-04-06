
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/organized_edge_detection.h>



template <typename PointT, typename PointLT>
void defineFeaturesOrganizedEdgeBase(py::module &m, std::string const & suffix) {
    using Class = OrganizedEdgeBase<PointT, PointLT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("depth_discon_threshold", &Class::getDepthDisconThreshold, &Class::setDepthDisconThreshold);
    cls.def_property("max_search_neighbors", &Class::getMaxSearchNeighbors, &Class::setMaxSearchNeighbors);
    cls.def_property("edge_type", &Class::getEdgeType, &Class::setEdgeType);
    cls.def_readonly_static("num_of_edgetype_", &Class::num_of_edgetype_);
    cls.def("compute", &Class::compute);
        
}

template <typename PointT, typename PointNT, typename PointLT>
void defineFeaturesOrganizedEdgeFromNormals(py::module &m, std::string const & suffix) {
    using Class = OrganizedEdgeFromNormals<PointT, PointNT, PointLT>;
    py::class_<Class, OrganizedEdgeBase<PointT,PointLT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("input_normals", &Class::getInputNormals, &Class::setInputNormals);
    cls.def_property("hc_canny_low_threshold", &Class::getHCCannyLowThreshold, &Class::setHCCannyLowThreshold);
    cls.def_property("hc_canny_high_threshold", &Class::getHCCannyHighThreshold, &Class::setHCCannyHighThreshold);
    cls.def("compute", &Class::compute);
        
}

template <typename PointT, typename PointLT>
void defineFeaturesOrganizedEdgeFromRGB(py::module &m, std::string const & suffix) {
    using Class = OrganizedEdgeFromRGB<PointT, PointLT>;
    py::class_<Class, OrganizedEdgeBase<PointT,PointLT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("rgb_canny_low_threshold", &Class::getRGBCannyLowThreshold, &Class::setRGBCannyLowThreshold);
    cls.def_property("rgb_canny_high_threshold", &Class::getRGBCannyHighThreshold, &Class::setRGBCannyHighThreshold);
    cls.def("compute", &Class::compute);
        
}

template <typename PointT, typename PointNT, typename PointLT>
void defineFeaturesOrganizedEdgeFromRGBNormals(py::module &m, std::string const & suffix) {
    using Class = OrganizedEdgeFromRGBNormals<PointT, PointNT, PointLT>;
    py::class_<Class, OrganizedEdgeFromRGB<PointT,PointLT>, OrganizedEdgeFromNormals<PointT,PointNT,PointLT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute", &Class::compute);
        
}

void defineFeaturesOrganizedEdgeDetectionClasses(py::module &sub_module) {
    py::module sub_module_OrganizedEdgeBase = sub_module.def_submodule("OrganizedEdgeBase", "Submodule OrganizedEdgeBase");
    defineFeaturesOrganizedEdgeBase<InterestPoint, Label>(sub_module_OrganizedEdgeBase, "InterestPoint_Label");
    defineFeaturesOrganizedEdgeBase<PointDEM, Label>(sub_module_OrganizedEdgeBase, "PointDEM_Label");
    defineFeaturesOrganizedEdgeBase<PointNormal, Label>(sub_module_OrganizedEdgeBase, "PointNormal_Label");
    defineFeaturesOrganizedEdgeBase<PointSurfel, Label>(sub_module_OrganizedEdgeBase, "PointSurfel_Label");
    defineFeaturesOrganizedEdgeBase<PointWithRange, Label>(sub_module_OrganizedEdgeBase, "PointWithRange_Label");
    defineFeaturesOrganizedEdgeBase<PointWithScale, Label>(sub_module_OrganizedEdgeBase, "PointWithScale_Label");
    defineFeaturesOrganizedEdgeBase<PointWithViewpoint, Label>(sub_module_OrganizedEdgeBase, "PointWithViewpoint_Label");
    defineFeaturesOrganizedEdgeBase<PointXYZ, Label>(sub_module_OrganizedEdgeBase, "PointXYZ_Label");
    defineFeaturesOrganizedEdgeBase<PointXYZHSV, Label>(sub_module_OrganizedEdgeBase, "PointXYZHSV_Label");
    defineFeaturesOrganizedEdgeBase<PointXYZI, Label>(sub_module_OrganizedEdgeBase, "PointXYZI_Label");
    defineFeaturesOrganizedEdgeBase<PointXYZINormal, Label>(sub_module_OrganizedEdgeBase, "PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeBase<PointXYZL, Label>(sub_module_OrganizedEdgeBase, "PointXYZL_Label");
    defineFeaturesOrganizedEdgeBase<PointXYZLNormal, Label>(sub_module_OrganizedEdgeBase, "PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeBase<PointXYZRGB, Label>(sub_module_OrganizedEdgeBase, "PointXYZRGB_Label");
    defineFeaturesOrganizedEdgeBase<PointXYZRGBA, Label>(sub_module_OrganizedEdgeBase, "PointXYZRGBA_Label");
    defineFeaturesOrganizedEdgeBase<PointXYZRGBL, Label>(sub_module_OrganizedEdgeBase, "PointXYZRGBL_Label");
    defineFeaturesOrganizedEdgeBase<PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeBase, "PointXYZRGBNormal_Label");
    py::module sub_module_OrganizedEdgeFromNormals = sub_module.def_submodule("OrganizedEdgeFromNormals", "Submodule OrganizedEdgeFromNormals");
    defineFeaturesOrganizedEdgeFromNormals<InterestPoint, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<InterestPoint, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<InterestPoint, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<InterestPoint, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<InterestPoint, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<InterestPoint, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "InterestPoint_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointDEM, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointDEM, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointDEM, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointDEM, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointDEM, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointDEM, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointDEM_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointNormal, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointNormal, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointNormal, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointNormal, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointNormal, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointNormal, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointNormal_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointSurfel, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointSurfel, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointSurfel, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointSurfel, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointSurfel, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointSurfel, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointSurfel_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithRange, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithRange, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithRange, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithRange, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithRange, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithRange, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithRange_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithScale, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithScale, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithScale, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithScale, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithScale, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithScale, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithScale_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithViewpoint, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithViewpoint, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithViewpoint, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithViewpoint, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithViewpoint, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointWithViewpoint, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointWithViewpoint_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZ, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZ, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZ, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZ, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZ, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZ, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZ_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZHSV, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZHSV, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZHSV, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZHSV, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZHSV, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZHSV, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZHSV_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZI, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZI, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZI, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZI, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZI, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZI, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZI_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZINormal, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZINormal, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZINormal, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZINormal, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZINormal, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZINormal, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZINormal_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZL, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZL, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZL, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZL, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZL, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZL, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZL_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZLNormal, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZLNormal, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZLNormal, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZLNormal, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZLNormal, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZLNormal, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZLNormal_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGB, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGB, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGB, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGB, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGB, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGB, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGB_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBA, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBA, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBA, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBA, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBA, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBA, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBA_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBL, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBL, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBL, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBL, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBL, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBL, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBL_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBNormal, Normal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_Normal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBNormal, PointNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBNormal, PointSurfel, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBNormal, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBNormal, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromNormals<PointXYZRGBNormal, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromNormals, "PointXYZRGBNormal_PointXYZRGBNormal_Label");
    py::module sub_module_OrganizedEdgeFromRGB = sub_module.def_submodule("OrganizedEdgeFromRGB", "Submodule OrganizedEdgeFromRGB");
    defineFeaturesOrganizedEdgeFromRGB<PointSurfel, Label>(sub_module_OrganizedEdgeFromRGB, "PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGB<PointXYZRGB, Label>(sub_module_OrganizedEdgeFromRGB, "PointXYZRGB_Label");
    defineFeaturesOrganizedEdgeFromRGB<PointXYZRGBA, Label>(sub_module_OrganizedEdgeFromRGB, "PointXYZRGBA_Label");
    defineFeaturesOrganizedEdgeFromRGB<PointXYZRGBL, Label>(sub_module_OrganizedEdgeFromRGB, "PointXYZRGBL_Label");
    defineFeaturesOrganizedEdgeFromRGB<PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromRGB, "PointXYZRGBNormal_Label");
    py::module sub_module_OrganizedEdgeFromRGBNormals = sub_module.def_submodule("OrganizedEdgeFromRGBNormals", "Submodule OrganizedEdgeFromRGBNormals");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointSurfel, Normal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_Normal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointSurfel, PointNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointSurfel, PointSurfel, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointSurfel, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointSurfel, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointSurfel, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointSurfel_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGB, Normal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_Normal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGB, PointNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGB, PointSurfel, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGB, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGB, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGB, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGB_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBA, Normal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_Normal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBA, PointNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBA, PointSurfel, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBA, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBA, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBA, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBA_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBL, Normal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_Normal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBL, PointNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBL, PointSurfel, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBL, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBL, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBL, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBL_PointXYZRGBNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBNormal, Normal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_Normal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBNormal, PointNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_PointNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBNormal, PointSurfel, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_PointSurfel_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBNormal, PointXYZINormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_PointXYZINormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBNormal, PointXYZLNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_PointXYZLNormal_Label");
    defineFeaturesOrganizedEdgeFromRGBNormals<PointXYZRGBNormal, PointXYZRGBNormal, Label>(sub_module_OrganizedEdgeFromRGBNormals, "PointXYZRGBNormal_PointXYZRGBNormal_Label");
}