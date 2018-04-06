
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_pointcloud_adjacency.h>

using namespace pcl::octree;


template <typename PointT,
              typename LeafContainerT = OctreePointCloudAdjacencyContainer<PointT>,
              typename BranchContainerT = OctreeContainerEmpty>
void defineOctreeOctreePointCloudAdjacency(py::module &m, std::string const & suffix) {
    using Class = octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>;
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
    py::class_<Class, OctreePointCloud<PointT,LeafContainerT,BranchContainerT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
    cls.def("set_transform_function", &Class::setTransformFunction);
    cls.def("depth_begin", &Class::depth_begin);
    cls.def("depth_end", &Class::depth_end);
    cls.def("leaf_begin", &Class::leaf_begin);
    cls.def("leaf_end", &Class::leaf_end);
    cls.def("begin", &Class::begin);
    cls.def("end", &Class::end);
    cls.def("at", &Class::at);
    cls.def("size", &Class::size);
    cls.def("add_points_from_input_cloud", &Class::addPointsFromInputCloud);
    cls.def("compute_voxel_adjacency_graph", &Class::computeVoxelAdjacencyGraph);
    cls.def("test_for_occlusion", &Class::testForOcclusion);
        
}

void defineOctreeOctreePointcloudAdjacencyClasses(py::module &sub_module) {
}