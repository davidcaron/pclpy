
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

#include <pcl/octree/octree_pointcloud_occupancy.h>

using namespace pcl::octree;


template<typename PointT,
             typename LeafContainerT = OctreeContainerEmpty,
             typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreePointCloudOccupancy(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreePointCloudOccupancy<PointT, LeafContainerT, BranchContainerT>;
    using SingleBuffer = Class::SingleBuffer;
    using DoubleBuffer = Class::DoubleBuffer;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    py::class_<Class, pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT, pcl::octree::OctreeBase<pcl::octree::LeafContainer>OctreeBase<LeafContainerT, pcl::octree::BranchContainerT>>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
    cls.def("setOccupiedVoxelAtPoint", &Class::setOccupiedVoxelAtPoint, "point_arg"_a);
    cls.def("setOccupiedVoxelsAtPointsFromCloud", &Class::setOccupiedVoxelsAtPointsFromCloud, "cloud_arg"_a);
        
}

void defineOctreeOctreePointcloudOccupancyFunctions(py::module &m) {
}

void defineOctreeOctreePointcloudOccupancyClasses(py::module &sub_module) {
}