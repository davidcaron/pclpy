
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

#include <pcl/segmentation/region_growing.h>



template <typename PointT, typename NormalT>
void defineSegmentationRegionGrowing(py::module &m, std::string const & suffix) {
    using Class = pcl::RegionGrowing<PointT, NormalT>;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using Normal = Class::Normal;
    using NormalPtr = Class::NormalPtr;
    using PointCloud = Class::PointCloud;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("extract", &Class::extract, "clusters"_a);
    cls.def("setMinClusterSize", &Class::setMinClusterSize, "min_cluster_size"_a);
    cls.def("setMaxClusterSize", &Class::setMaxClusterSize, "max_cluster_size"_a);
    cls.def("setSmoothModeFlag", &Class::setSmoothModeFlag, "value"_a);
    cls.def("setCurvatureTestFlag", &Class::setCurvatureTestFlag, "value"_a);
    cls.def("setResidualTestFlag", &Class::setResidualTestFlag, "value"_a);
    cls.def("setSmoothnessThreshold", &Class::setSmoothnessThreshold, "theta"_a);
    cls.def("setResidualThreshold", &Class::setResidualThreshold, "residual"_a);
    cls.def("setCurvatureThreshold", &Class::setCurvatureThreshold, "curvature"_a);
    cls.def("setNumberOfNeighbours", &Class::setNumberOfNeighbours, "neighbour_number"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("setInputNormals", &Class::setInputNormals, "norm"_a);
    cls.def("getMinClusterSize", &Class::getMinClusterSize);
    cls.def("getMaxClusterSize", &Class::getMaxClusterSize);
    cls.def("getSmoothModeFlag", &Class::getSmoothModeFlag);
    cls.def("getCurvatureTestFlag", &Class::getCurvatureTestFlag);
    cls.def("getResidualTestFlag", &Class::getResidualTestFlag);
    cls.def("getSmoothnessThreshold", &Class::getSmoothnessThreshold);
    cls.def("getResidualThreshold", &Class::getResidualThreshold);
    cls.def("getCurvatureThreshold", &Class::getCurvatureThreshold);
    cls.def("getNumberOfNeighbours", &Class::getNumberOfNeighbours);
    cls.def("getSearchMethod", &Class::getSearchMethod);
    cls.def("getInputNormals", &Class::getInputNormals);
    cls.def("getSegmentFromPoint", &Class::getSegmentFromPoint, "index"_a, "cluster"_a);
    cls.def("getColoredCloud", &Class::getColoredCloud);
    cls.def("getColoredCloudRGBA", &Class::getColoredCloudRGBA);
        
}

void defineSegmentationRegionGrowingFunctions1(py::module &m) {
    m.def("comparePair", py::overload_cast<std::pair<float, int>, std::pair<float, int>> (&pcl::comparePair), "i"_a, "j"_a);
}

void defineSegmentationRegionGrowingFunctions(py::module &m) {
    defineSegmentationRegionGrowingFunctions1(m);
}

void defineSegmentationRegionGrowingClasses(py::module &sub_module) {
    py::module sub_module_RegionGrowing = sub_module.def_submodule("RegionGrowing", "Submodule RegionGrowing");
    defineSegmentationRegionGrowing<pcl::InterestPoint, pcl::Normal>(sub_module_RegionGrowing, "InterestPoint_Normal");
    defineSegmentationRegionGrowing<pcl::PointDEM, pcl::Normal>(sub_module_RegionGrowing, "PointDEM_Normal");
    defineSegmentationRegionGrowing<pcl::PointNormal, pcl::Normal>(sub_module_RegionGrowing, "PointNormal_Normal");
    defineSegmentationRegionGrowing<pcl::PointSurfel, pcl::Normal>(sub_module_RegionGrowing, "PointSurfel_Normal");
    defineSegmentationRegionGrowing<pcl::PointWithRange, pcl::Normal>(sub_module_RegionGrowing, "PointWithRange_Normal");
    defineSegmentationRegionGrowing<pcl::PointWithScale, pcl::Normal>(sub_module_RegionGrowing, "PointWithScale_Normal");
    defineSegmentationRegionGrowing<pcl::PointWithViewpoint, pcl::Normal>(sub_module_RegionGrowing, "PointWithViewpoint_Normal");
    defineSegmentationRegionGrowing<pcl::PointXYZ, pcl::Normal>(sub_module_RegionGrowing, "PointXYZ_Normal");
    defineSegmentationRegionGrowing<pcl::PointXYZHSV, pcl::Normal>(sub_module_RegionGrowing, "PointXYZHSV_Normal");
    defineSegmentationRegionGrowing<pcl::PointXYZI, pcl::Normal>(sub_module_RegionGrowing, "PointXYZI_Normal");
    defineSegmentationRegionGrowing<pcl::PointXYZINormal, pcl::Normal>(sub_module_RegionGrowing, "PointXYZINormal_Normal");
    defineSegmentationRegionGrowing<pcl::PointXYZL, pcl::Normal>(sub_module_RegionGrowing, "PointXYZL_Normal");
    defineSegmentationRegionGrowing<pcl::PointXYZLNormal, pcl::Normal>(sub_module_RegionGrowing, "PointXYZLNormal_Normal");
    defineSegmentationRegionGrowing<pcl::PointXYZRGB, pcl::Normal>(sub_module_RegionGrowing, "PointXYZRGB_Normal");
    defineSegmentationRegionGrowing<pcl::PointXYZRGBA, pcl::Normal>(sub_module_RegionGrowing, "PointXYZRGBA_Normal");
    defineSegmentationRegionGrowing<pcl::PointXYZRGBL, pcl::Normal>(sub_module_RegionGrowing, "PointXYZRGBL_Normal");
    defineSegmentationRegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal>(sub_module_RegionGrowing, "PointXYZRGBNormal_Normal");
    defineSegmentationRegionGrowingFunctions(sub_module);
}