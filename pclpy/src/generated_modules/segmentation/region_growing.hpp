
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/region_growing.h>



template <typename PointT, typename NormalT>
void defineSegmentationRegionGrowing(py::module &m, std::string const & suffix) {
    using Class = RegionGrowing<PointT, NormalT>;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using Normal = Class::Normal;
    using NormalPtr = Class::NormalPtr;
    using PointCloud = Class::PointCloud;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("min_cluster_size", &Class::getMinClusterSize, &Class::setMinClusterSize);
    cls.def_property("max_cluster_size", &Class::getMaxClusterSize, &Class::setMaxClusterSize);
    cls.def_property("smooth_mode_flag", &Class::getSmoothModeFlag, &Class::setSmoothModeFlag);
    cls.def_property("curvature_test_flag", &Class::getCurvatureTestFlag, &Class::setCurvatureTestFlag);
    cls.def_property("residual_test_flag", &Class::getResidualTestFlag, &Class::setResidualTestFlag);
    cls.def_property("smoothness_threshold", &Class::getSmoothnessThreshold, &Class::setSmoothnessThreshold);
    cls.def_property("residual_threshold", &Class::getResidualThreshold, &Class::setResidualThreshold);
    cls.def_property("curvature_threshold", &Class::getCurvatureThreshold, &Class::setCurvatureThreshold);
    cls.def_property("number_of_neighbours", &Class::getNumberOfNeighbours, &Class::setNumberOfNeighbours);
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def_property("input_normals", &Class::getInputNormals, &Class::setInputNormals);
    cls.def("extract", &Class::extract);
        
}

void defineSegmentationRegionGrowingClasses(py::module &sub_module) {
    py::module sub_module_RegionGrowing = sub_module.def_submodule("RegionGrowing", "Submodule RegionGrowing");
    defineSegmentationRegionGrowing<InterestPoint, Normal>(sub_module_RegionGrowing, "InterestPoint_Normal");
    defineSegmentationRegionGrowing<PointDEM, Normal>(sub_module_RegionGrowing, "PointDEM_Normal");
    defineSegmentationRegionGrowing<PointNormal, Normal>(sub_module_RegionGrowing, "PointNormal_Normal");
    defineSegmentationRegionGrowing<PointSurfel, Normal>(sub_module_RegionGrowing, "PointSurfel_Normal");
    defineSegmentationRegionGrowing<PointWithRange, Normal>(sub_module_RegionGrowing, "PointWithRange_Normal");
    defineSegmentationRegionGrowing<PointWithScale, Normal>(sub_module_RegionGrowing, "PointWithScale_Normal");
    defineSegmentationRegionGrowing<PointWithViewpoint, Normal>(sub_module_RegionGrowing, "PointWithViewpoint_Normal");
    defineSegmentationRegionGrowing<PointXYZ, Normal>(sub_module_RegionGrowing, "PointXYZ_Normal");
    defineSegmentationRegionGrowing<PointXYZHSV, Normal>(sub_module_RegionGrowing, "PointXYZHSV_Normal");
    defineSegmentationRegionGrowing<PointXYZI, Normal>(sub_module_RegionGrowing, "PointXYZI_Normal");
    defineSegmentationRegionGrowing<PointXYZINormal, Normal>(sub_module_RegionGrowing, "PointXYZINormal_Normal");
    defineSegmentationRegionGrowing<PointXYZL, Normal>(sub_module_RegionGrowing, "PointXYZL_Normal");
    defineSegmentationRegionGrowing<PointXYZLNormal, Normal>(sub_module_RegionGrowing, "PointXYZLNormal_Normal");
    defineSegmentationRegionGrowing<PointXYZRGB, Normal>(sub_module_RegionGrowing, "PointXYZRGB_Normal");
    defineSegmentationRegionGrowing<PointXYZRGBA, Normal>(sub_module_RegionGrowing, "PointXYZRGBA_Normal");
    defineSegmentationRegionGrowing<PointXYZRGBL, Normal>(sub_module_RegionGrowing, "PointXYZRGBL_Normal");
    defineSegmentationRegionGrowing<PointXYZRGBNormal, Normal>(sub_module_RegionGrowing, "PointXYZRGBNormal_Normal");
}