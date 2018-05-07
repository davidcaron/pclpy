
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

using namespace pcl::octree;


template<typename PointT,
             typename LeafContainerT = OctreePointCloudVoxelCentroidContainer<PointT> ,
             typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreePointCloudVoxelCentroid(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreePointCloudVoxelCentroid<PointT, LeafContainerT, BranchContainerT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using OctreeT = Class::OctreeT;
    using LeafNode = Class::LeafNode;
    using BranchNode = Class::BranchNode;
    py::class_<Class, pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
    cls.def("addPointIdx", &Class::addPointIdx, "pointIdx_arg"_a);
    cls.def("getVoxelCentroidAtPoint", py::overload_cast<const PointT &, PointT &> (&Class::getVoxelCentroidAtPoint, py::const_), "point_arg"_a, "voxel_centroid_arg"_a);
    cls.def("getVoxelCentroidAtPoint", py::overload_cast<const int &, PointT &> (&Class::getVoxelCentroidAtPoint, py::const_), "point_idx_arg"_a, "voxel_centroid_arg"_a);
    cls.def("getVoxelCentroids", &Class::getVoxelCentroids, "voxel_centroid_list_arg"_a);
    cls.def("getVoxelCentroidsRecursive", &Class::getVoxelCentroidsRecursive, "branch_arg"_a, "key_arg"_a, "voxel_centroid_list_arg"_a);
        
}

template<typename PointT>
void defineOctreeOctreePointCloudVoxelCentroidContainer(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreePointCloudVoxelCentroidContainer<PointT>;
    py::class_<Class, pcl::octree::OctreeContainerBase, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("deepCopy", &Class::deepCopy);
    // Operators not implemented (operator==);
    cls.def("addPoint", &Class::addPoint, "new_point"_a);
    cls.def("reset", &Class::reset);
    cls.def("getCentroid", &Class::getCentroid, "centroid_arg"_a);
        
}

void defineOctreeOctreePointcloudVoxelcentroidFunctions(py::module &m) {
}

void defineOctreeOctreePointcloudVoxelcentroidClasses(py::module &sub_module) {
    py::module sub_module_OctreePointCloudVoxelCentroid = sub_module.def_submodule("OctreePointCloudVoxelCentroid", "Submodule OctreePointCloudVoxelCentroid");
    defineOctreeOctreePointCloudVoxelCentroid<pcl::PointXYZ>(sub_module_OctreePointCloudVoxelCentroid, "PointXYZ");
    defineOctreeOctreePointCloudVoxelCentroid<pcl::PointXYZI>(sub_module_OctreePointCloudVoxelCentroid, "PointXYZI");
    defineOctreeOctreePointCloudVoxelCentroid<pcl::PointXYZL>(sub_module_OctreePointCloudVoxelCentroid, "PointXYZL");
    defineOctreeOctreePointCloudVoxelCentroid<pcl::PointXYZRGB>(sub_module_OctreePointCloudVoxelCentroid, "PointXYZRGB");
    defineOctreeOctreePointCloudVoxelCentroid<pcl::PointXYZRGBA>(sub_module_OctreePointCloudVoxelCentroid, "PointXYZRGBA");
    py::module sub_module_OctreePointCloudVoxelCentroidContainer = sub_module.def_submodule("OctreePointCloudVoxelCentroidContainer", "Submodule OctreePointCloudVoxelCentroidContainer");
    defineOctreeOctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>(sub_module_OctreePointCloudVoxelCentroidContainer, "PointXYZ");
    defineOctreeOctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>(sub_module_OctreePointCloudVoxelCentroidContainer, "PointXYZI");
    defineOctreeOctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>(sub_module_OctreePointCloudVoxelCentroidContainer, "PointXYZL");
    defineOctreeOctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>(sub_module_OctreePointCloudVoxelCentroidContainer, "PointXYZRGB");
    defineOctreeOctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>(sub_module_OctreePointCloudVoxelCentroidContainer, "PointXYZRGBA");
}