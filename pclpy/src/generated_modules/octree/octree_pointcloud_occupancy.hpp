
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_pointcloud_occupancy.h>

using namespace pcl::octree;


template<typename PointT,
             typename LeafContainerT = OctreeContainerEmpty,
             typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreePointCloudOccupancy(py::module &m, std::string const & suffix) {
    using Class = octree::OctreePointCloudOccupancy<PointT, LeafContainerT, BranchContainerT>;
    using SingleBuffer = Class::SingleBuffer;
    using DoubleBuffer = Class::DoubleBuffer;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    py::class_<Class, OctreePointCloud<PointT,LeafContainerT,BranchContainerT,OctreeBase<LeafContainerT,BranchContainerT>>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
    cls.def("set_occupied_voxel_at_point", &Class::setOccupiedVoxelAtPoint);
    cls.def("set_occupied_voxels_at_points_from_cloud", &Class::setOccupiedVoxelsAtPointsFromCloud);
        
}

void defineOctreeOctreePointcloudOccupancyClasses(py::module &sub_module) {
}