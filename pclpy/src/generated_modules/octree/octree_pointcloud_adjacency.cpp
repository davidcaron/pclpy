
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

#include <pcl/octree/octree_pointcloud_adjacency.h>

using namespace pcl::octree;


template <typename PointT,
              typename LeafContainerT = OctreePointCloudAdjacencyContainer<PointT>,
              typename BranchContainerT = OctreeContainerEmpty>
void defineOctreeOctreePointCloudAdjacency(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>;
    using OctreeBaseT = Class::OctreeBaseT;
    using OctreeAdjacencyT = Class::OctreeAdjacencyT;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using OctreePointCloudT = Class::OctreePointCloudT;
    using LeafNode = Class::LeafNode;
    using BranchNode = Class::BranchNode;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Iterator = Class::Iterator;
    using ConstIterator = Class::ConstIterator;
    using LeafNodeIterator = Class::LeafNodeIterator;
    using ConstLeafNodeIterator = Class::ConstLeafNodeIterator;
    using VoxelAdjacencyList = Class::VoxelAdjacencyList;
    using VoxelID = Class::VoxelID;
    using EdgeID = Class::EdgeID;
    using LeafVectorT = Class::LeafVectorT;
    using iterator = Class::iterator;
    using const_iterator = Class::const_iterator;
    py::class_<Class, pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
    cls.def("depth_begin", &Class::depth_begin, "max_depth_arg"_a=0);
    cls.def("depth_end", &Class::depth_end);
    cls.def("leaf_begin", &Class::leaf_begin, "max_depth_arg"_a=0);
    cls.def("leaf_end", &Class::leaf_end);
    cls.def("begin", &Class::begin);
    cls.def("end", &Class::end);
    cls.def("at", &Class::at, "idx"_a);
    cls.def("size", &Class::size);
    cls.def("addPointsFromInputCloud", &Class::addPointsFromInputCloud);
    cls.def("computeVoxelAdjacencyGraph", &Class::computeVoxelAdjacencyGraph, "voxel_adjacency_graph"_a);
    cls.def("testForOcclusion", &Class::testForOcclusion, "point_arg"_a, "camera_pos"_a=pcl::PointXYZ(0,0,0));
    cls.def("setTransformFunction", &Class::setTransformFunction, "transform_func"_a);
    cls.def("getLeafContainerAtPoint", &Class::getLeafContainerAtPoint, "point_arg"_a);
        
}

void defineOctreeOctreePointcloudAdjacencyFunctions(py::module &m) {
}

void defineOctreeOctreePointcloudAdjacencyClasses(py::module &sub_module) {
}