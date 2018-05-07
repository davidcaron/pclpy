
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud.h>

using namespace pcl::octree;


template<typename PointT, typename LeafContainerT = OctreeContainerPointIndices,
        typename BranchContainerT = OctreeContainerEmpty,
        typename OctreeT = OctreeBase<LeafContainerT, BranchContainerT> >
void defineOctreeOctreePointCloud(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>;
    using Base = Class::Base;
    using LeafNode = Class::LeafNode;
    using BranchNode = Class::BranchNode;
    using Iterator = Class::Iterator;
    using ConstIterator = Class::ConstIterator;
    using LeafNodeIterator = Class::LeafNodeIterator;
    using ConstLeafNodeIterator = Class::ConstLeafNodeIterator;
    using DepthFirstIterator = Class::DepthFirstIterator;
    using ConstDepthFirstIterator = Class::ConstDepthFirstIterator;
    using BreadthFirstIterator = Class::BreadthFirstIterator;
    using ConstBreadthFirstIterator = Class::ConstBreadthFirstIterator;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using SingleBuffer = Class::SingleBuffer;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using AlignedPointTVector = Class::AlignedPointTVector;
    using AlignedPointXYZVector = Class::AlignedPointXYZVector;
    py::class_<Class, OctreeT, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
    cls.def("addPointsFromInputCloud", &Class::addPointsFromInputCloud);
    cls.def("addPointFromCloud", &Class::addPointFromCloud, "point_idx_arg"_a, "indices_arg"_a);
    cls.def("addPointToCloud", py::overload_cast<const PointT &, PointCloudPtr> (&Class::addPointToCloud), "point_arg"_a, "cloud_arg"_a);
    cls.def("addPointToCloud", py::overload_cast<const PointT &, PointCloudPtr, pcl::IndicesPtr> (&Class::addPointToCloud), "point_arg"_a, "cloud_arg"_a, "indices_arg"_a);
    cls.def("isVoxelOccupiedAtPoint", py::overload_cast<const PointT &> (&Class::isVoxelOccupiedAtPoint, py::const_), "point_arg"_a);
    cls.def("deleteTree", &Class::deleteTree);
    cls.def("isVoxelOccupiedAtPoint", py::overload_cast<const double, const double, const double> (&Class::isVoxelOccupiedAtPoint, py::const_), "point_x_arg"_a, "point_y_arg"_a, "point_z_arg"_a);
    cls.def("isVoxelOccupiedAtPoint", py::overload_cast<const int &> (&Class::isVoxelOccupiedAtPoint, py::const_), "point_idx_arg"_a);
    cls.def("deleteVoxelAtPoint", py::overload_cast<const PointT &> (&Class::deleteVoxelAtPoint), "point_arg"_a);
    cls.def("deleteVoxelAtPoint", py::overload_cast<const int &> (&Class::deleteVoxelAtPoint), "point_idx_arg"_a);
    cls.def("defineBoundingBox", py::overload_cast<> (&Class::defineBoundingBox));
    cls.def("defineBoundingBox", py::overload_cast<const double, const double, const double, const double, const double, const double> (&Class::defineBoundingBox), "min_x_arg"_a, "min_y_arg"_a, "min_z_arg"_a, "max_x_arg"_a, "max_y_arg"_a, "max_z_arg"_a);
    cls.def("defineBoundingBox", py::overload_cast<const double, const double, const double> (&Class::defineBoundingBox), "max_x_arg"_a, "max_y_arg"_a, "max_z_arg"_a);
    cls.def("defineBoundingBox", py::overload_cast<const double> (&Class::defineBoundingBox), "cubeLen_arg"_a);
    cls.def("enableDynamicDepth", &Class::enableDynamicDepth, "maxObjsPerLeaf"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud_arg"_a, "indices_arg"_a=pcl::IndicesConstPtr());
    cls.def("setEpsilon", &Class::setEpsilon, "eps"_a);
    cls.def("setResolution", &Class::setResolution, "resolution_arg"_a);
    cls.def("getIndices", &Class::getIndices);
    cls.def("getInputCloud", &Class::getInputCloud);
    cls.def("getEpsilon", &Class::getEpsilon);
    cls.def("getResolution", &Class::getResolution);
    cls.def("getTreeDepth", &Class::getTreeDepth);
    cls.def("getOccupiedVoxelCenters", &Class::getOccupiedVoxelCenters, "voxel_center_list_arg"_a);
    cls.def("getApproxIntersectedVoxelCentersBySegment", &Class::getApproxIntersectedVoxelCentersBySegment, "origin"_a, "end"_a, "voxel_center_list"_a, "precision"_a=0.2);
    cls.def("getBoundingBox", &Class::getBoundingBox, "min_x_arg"_a, "min_y_arg"_a, "min_z_arg"_a, "max_x_arg"_a, "max_y_arg"_a, "max_z_arg"_a);
    cls.def("getVoxelSquaredDiameter", py::overload_cast<unsigned int> (&Class::getVoxelSquaredDiameter, py::const_), "tree_depth_arg"_a);
    cls.def("getVoxelSquaredDiameter", py::overload_cast<> (&Class::getVoxelSquaredDiameter, py::const_));
    cls.def("getVoxelSquaredSideLen", py::overload_cast<unsigned int> (&Class::getVoxelSquaredSideLen, py::const_), "tree_depth_arg"_a);
    cls.def("getVoxelSquaredSideLen", py::overload_cast<> (&Class::getVoxelSquaredSideLen, py::const_));
    cls.def("getVoxelBounds", &Class::getVoxelBounds, "iterator"_a, "min_pt"_a, "max_pt"_a);
        
}

void defineOctreeOctreePointcloudFunctions(py::module &m) {
}

void defineOctreeOctreePointcloudClasses(py::module &sub_module) {
    py::module sub_module_OctreePointCloud = sub_module.def_submodule("OctreePointCloud", "Submodule OctreePointCloud");
    defineOctreeOctreePointCloud<pcl::InterestPoint>(sub_module_OctreePointCloud, "InterestPoint");
    defineOctreeOctreePointCloud<pcl::PointDEM>(sub_module_OctreePointCloud, "PointDEM");
    defineOctreeOctreePointCloud<pcl::PointNormal>(sub_module_OctreePointCloud, "PointNormal");
    defineOctreeOctreePointCloud<pcl::PointSurfel>(sub_module_OctreePointCloud, "PointSurfel");
    defineOctreeOctreePointCloud<pcl::PointWithRange>(sub_module_OctreePointCloud, "PointWithRange");
    defineOctreeOctreePointCloud<pcl::PointWithScale>(sub_module_OctreePointCloud, "PointWithScale");
    defineOctreeOctreePointCloud<pcl::PointWithViewpoint>(sub_module_OctreePointCloud, "PointWithViewpoint");
    defineOctreeOctreePointCloud<pcl::PointXYZ>(sub_module_OctreePointCloud, "PointXYZ");
    defineOctreeOctreePointCloud<pcl::PointXYZ, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZ_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZ, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZ_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZ, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZ_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZ, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZ_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZ, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZ_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZHSV>(sub_module_OctreePointCloud, "PointXYZHSV");
    defineOctreeOctreePointCloud<pcl::PointXYZI>(sub_module_OctreePointCloud, "PointXYZI");
    defineOctreeOctreePointCloud<pcl::PointXYZI, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZI_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZI, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZI_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZI, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZI_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZI, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZI_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZI, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZI_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZINormal>(sub_module_OctreePointCloud, "PointXYZINormal");
    defineOctreeOctreePointCloud<pcl::PointXYZL>(sub_module_OctreePointCloud, "PointXYZL");
    defineOctreeOctreePointCloud<pcl::PointXYZL, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZL_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZL, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZL_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZL, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZL_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZL, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZL_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZL, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZL_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZLNormal>(sub_module_OctreePointCloud, "PointXYZLNormal");
    defineOctreeOctreePointCloud<pcl::PointXYZRGB>(sub_module_OctreePointCloud, "PointXYZRGB");
    defineOctreeOctreePointCloud<pcl::PointXYZRGB, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZRGB_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZRGB, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZRGB_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZRGB, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZRGB_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZRGB, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZRGB_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZRGB, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZRGB_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZRGBA>(sub_module_OctreePointCloud, "PointXYZRGBA");
    defineOctreeOctreePointCloud<pcl::PointXYZRGBA, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZRGBA_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZRGBA, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZRGBA_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZRGBA, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZRGBA_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZRGBA, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZRGBA_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZRGBA, pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreePointCloud, "PointXYZRGBA_octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>_octree::OctreeContainerEmpty");
    defineOctreeOctreePointCloud<pcl::PointXYZRGBL>(sub_module_OctreePointCloud, "PointXYZRGBL");
    defineOctreeOctreePointCloud<pcl::PointXYZRGBNormal>(sub_module_OctreePointCloud, "PointXYZRGBNormal");
}