
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/organized_multi_plane_segmentation.h>



template<typename PointT, typename PointNT, typename PointLT>
void defineSegmentationOrganizedMultiPlaneSegmentation(py::module &m, std::string const & suffix) {
    using Class = OrganizedMultiPlaneSegmentation<PointT, PointNT, PointLT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using PointCloudL = Class::PointCloudL;
    using PointCloudLPtr = Class::PointCloudLPtr;
    using PointCloudLConstPtr = Class::PointCloudLConstPtr;
    using PlaneComparator = Class::PlaneComparator;
    using PlaneComparatorPtr = Class::PlaneComparatorPtr;
    using PlaneComparatorConstPtr = Class::PlaneComparatorConstPtr;
    using PlaneRefinementComparator = Class::PlaneRefinementComparator;
    using PlaneRefinementComparatorPtr = Class::PlaneRefinementComparatorPtr;
    using PlaneRefinementComparatorConstPtr = Class::PlaneRefinementComparatorConstPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("input_normals", &Class::getInputNormals, &Class::setInputNormals);
    cls.def_property("min_inliers", &Class::getMinInliers, &Class::setMinInliers);
    cls.def_property("angular_threshold", &Class::getAngularThreshold, &Class::setAngularThreshold);
    cls.def_property("distance_threshold", &Class::getDistanceThreshold, &Class::setDistanceThreshold);
    cls.def_property("maximum_curvature", &Class::getMaximumCurvature, &Class::setMaximumCurvature);
    cls.def("set_comparator", &Class::setComparator);
    cls.def("set_refinement_comparator", &Class::setRefinementComparator);
    cls.def("set_project_points", &Class::setProjectPoints);
    cls.def("segment", py::overload_cast<std::vector<ModelCoefficients> &, std::vector<PointIndices> &, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &, std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > &, pcl::PointCloud<PointLT> &, std::vector<pcl::PointIndices> &> (&Class::segment));
    cls.def("segment", py::overload_cast<std::vector<ModelCoefficients> &, std::vector<PointIndices> &> (&Class::segment));
    cls.def("segment", py::overload_cast<std::vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT> > > &> (&Class::segment));
    cls.def("segment_and_refine", py::overload_cast<std::vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT> > > &> (&Class::segmentAndRefine));
    cls.def("segment_and_refine", py::overload_cast<std::vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT> > > &, std::vector<ModelCoefficients> &, std::vector<PointIndices> &, PointCloudLPtr &, std::vector<pcl::PointIndices> &, std::vector<pcl::PointIndices> &> (&Class::segmentAndRefine));
    cls.def("refine", &Class::refine);
        
}

void defineSegmentationOrganizedMultiPlaneSegmentationClasses(py::module &sub_module) {
    py::module sub_module_OrganizedMultiPlaneSegmentation = sub_module.def_submodule("OrganizedMultiPlaneSegmentation", "Submodule OrganizedMultiPlaneSegmentation");
    defineSegmentationOrganizedMultiPlaneSegmentation<InterestPoint, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<InterestPoint, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<InterestPoint, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<InterestPoint, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<InterestPoint, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<InterestPoint, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointDEM, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointDEM, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointDEM, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointDEM, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointDEM, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointDEM, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointNormal, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointNormal, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointNormal, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointNormal, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointNormal, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointNormal, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointSurfel, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointSurfel, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointSurfel, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointSurfel, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointSurfel, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointSurfel, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithRange, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithRange, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithRange, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithRange, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithRange, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithRange, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithScale, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithScale, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithScale, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithScale, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithScale, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithScale, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithViewpoint, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithViewpoint, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithViewpoint, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithViewpoint, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithViewpoint, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointWithViewpoint, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZ, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZ, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZ, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZ, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZ, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZ, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZHSV, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZHSV, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZHSV, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZHSV, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZHSV, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZHSV, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZI, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZI, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZI, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZI, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZI, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZI, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZINormal, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZINormal, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZINormal, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZINormal, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZINormal, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZINormal, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZL, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZL, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZL, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZL, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZL, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZL, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZLNormal, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZLNormal, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZLNormal, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZLNormal, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZLNormal, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZLNormal, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGB, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGB, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGB, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGB, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGB, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGB, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBA, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBA, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBA, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBA, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBA, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBA, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBL, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBL, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBL, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBL, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBL, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBL, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBNormal, Normal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBNormal, PointNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBNormal, PointSurfel, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBNormal, PointXYZINormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBNormal, PointXYZLNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<PointXYZRGBNormal, PointXYZRGBNormal, Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_PointXYZRGBNormal_Label");
}