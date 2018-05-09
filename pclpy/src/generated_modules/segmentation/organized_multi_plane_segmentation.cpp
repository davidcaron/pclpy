
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/segmentation/organized_multi_plane_segmentation.h>



template<typename PointT, typename PointNT, typename PointLT>
void defineSegmentationOrganizedMultiPlaneSegmentation(py::module &m, std::string const & suffix) {
    using Class = pcl::OrganizedMultiPlaneSegmentation<PointT, PointNT, PointLT>;
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
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("segment", py::overload_cast<std::vector<pcl::ModelCoefficients> &, std::vector<pcl::PointIndices> &, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &, std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > &, pcl::PointCloud<PointLT> &, std::vector<pcl::PointIndices> &> (&Class::segment), "model_coefficients"_a, "inlier_indices"_a, "centroids"_a, "covariances"_a, "labels"_a, "label_indices"_a);
    cls.def("segment", py::overload_cast<std::vector<pcl::ModelCoefficients> &, std::vector<pcl::PointIndices> &> (&Class::segment), "model_coefficients"_a, "inlier_indices"_a);
    cls.def("segment", py::overload_cast<std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &> (&Class::segment), "regions"_a);
    cls.def("segmentAndRefine", py::overload_cast<std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &> (&Class::segmentAndRefine), "regions"_a);
    cls.def("segmentAndRefine", py::overload_cast<std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &, std::vector<pcl::ModelCoefficients> &, std::vector<pcl::PointIndices> &, PointCloudLPtr &, std::vector<pcl::PointIndices> &, std::vector<pcl::PointIndices> &> (&Class::segmentAndRefine), "regions"_a, "model_coefficients"_a, "inlier_indices"_a, "labels"_a, "label_indices"_a, "boundary_indices"_a);
    cls.def("refine", &Class::refine, "model_coefficients"_a, "inlier_indices"_a, "centroids"_a, "covariances"_a, "labels"_a, "label_indices"_a);
    cls.def("setInputNormals", &Class::setInputNormals, "normals"_a);
    cls.def("setMinInliers", &Class::setMinInliers, "min_inliers"_a);
    cls.def("setAngularThreshold", &Class::setAngularThreshold, "angular_threshold"_a);
    cls.def("setDistanceThreshold", &Class::setDistanceThreshold, "distance_threshold"_a);
    cls.def("setMaximumCurvature", &Class::setMaximumCurvature, "maximum_curvature"_a);
    cls.def("setComparator", &Class::setComparator, "compare"_a);
    cls.def("setRefinementComparator", &Class::setRefinementComparator, "compare"_a);
    cls.def("setProjectPoints", &Class::setProjectPoints, "project_points"_a);
    cls.def("getInputNormals", &Class::getInputNormals);
    cls.def("getMinInliers", &Class::getMinInliers);
    cls.def("getAngularThreshold", &Class::getAngularThreshold);
    cls.def("getDistanceThreshold", &Class::getDistanceThreshold);
    cls.def("getMaximumCurvature", &Class::getMaximumCurvature);
        
}

void defineSegmentationOrganizedMultiPlaneSegmentationFunctions(py::module &m) {
}

void defineSegmentationOrganizedMultiPlaneSegmentationClasses(py::module &sub_module) {
    py::module sub_module_OrganizedMultiPlaneSegmentation = sub_module.def_submodule("OrganizedMultiPlaneSegmentation", "Submodule OrganizedMultiPlaneSegmentation");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::InterestPoint, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::InterestPoint, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::InterestPoint, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::InterestPoint, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::InterestPoint, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::InterestPoint, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "InterestPoint_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointDEM, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointDEM, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointDEM, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointDEM, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointDEM, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointDEM, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointDEM_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointNormal, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointNormal, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointNormal, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointNormal, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointNormal, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointNormal, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointNormal_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointSurfel, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointSurfel, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointSurfel, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointSurfel, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointSurfel, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointSurfel, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointSurfel_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithRange, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithRange, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithRange, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithRange, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithRange, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithRange, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithRange_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithScale, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithScale, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithScale, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithScale, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithScale, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithScale, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithScale_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithViewpoint, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithViewpoint, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithViewpoint, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithViewpoint, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithViewpoint, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointWithViewpoint, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointWithViewpoint_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZ_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZHSV, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZHSV, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZHSV, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZHSV, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZHSV, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZHSV, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZHSV_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZI, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZI, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZI, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZI, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZI, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZI, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZI_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZINormal, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZINormal, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZINormal, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZINormal, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZINormal, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZINormal_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZL, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZL, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZL, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZL, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZL, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZL, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZL_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZLNormal, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZLNormal, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZLNormal, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZLNormal, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZLNormal, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZLNormal, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZLNormal_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGB_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBA_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBL, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBL, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBL, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBL, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBL, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBL, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBL_PointXYZRGBNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBNormal, pcl::Normal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_Normal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBNormal, pcl::PointNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_PointNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBNormal, pcl::PointSurfel, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_PointSurfel_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBNormal, pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_PointXYZINormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBNormal, pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_PointXYZLNormal_Label");
    defineSegmentationOrganizedMultiPlaneSegmentation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedMultiPlaneSegmentation, "PointXYZRGBNormal_PointXYZRGBNormal_Label");
}