
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

using namespace pcl::octree;


template<typename PointT,
             typename LeafContainerT = OctreePointCloudVoxelCentroidContainer<PointT> ,
             typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreePointCloudVoxelCentroid(py::module &m, std::string const & suffix) {
    using Class = octree::OctreePointCloudVoxelCentroid<PointT, LeafContainerT, BranchContainerT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using OctreeT = Class::OctreeT;
    using LeafNode = Class::LeafNode;
    using BranchNode = Class::BranchNode;
    py::class_<Class, OctreePointCloud<PointT,LeafContainerT,BranchContainerT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
    cls.def("add_point_idx", &Class::addPointIdx);
    cls.def("get_voxel_centroid_at_point", py::overload_cast<const PointT &, PointT &> (&Class::getVoxelCentroidAtPoint, py::const_));
    cls.def("get_voxel_centroid_at_point", py::overload_cast<const int &, PointT &> (&Class::getVoxelCentroidAtPoint, py::const_));
        
}

template<typename PointT>
void defineOctreeOctreePointCloudVoxelCentroidContainer(py::module &m, std::string const & suffix) {
    using Class = octree::OctreePointCloudVoxelCentroidContainer<PointT>;
    py::class_<Class, OctreeContainerBase, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("deep_copy", &Class::deepCopy);
    // Operators not implemented (operator==);
    cls.def("add_point", &Class::addPoint);
    cls.def("reset", &Class::reset);
        
}

void defineOctreeOctreePointcloudVoxelcentroidClasses(py::module &sub_module) {
}