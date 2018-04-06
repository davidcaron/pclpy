
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_pointcloud_changedetector.h>

using namespace pcl::octree;


template<typename PointT,
        typename LeafContainerT = OctreeContainerPointIndices,
        typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreePointCloudChangeDetector(py::module &m, std::string const & suffix) {
    using Class = octree::OctreePointCloudChangeDetector<PointT, LeafContainerT, BranchContainerT>;
    py::class_<Class, OctreePointCloud<PointT,LeafContainerT,BranchContainerT,Octree2BufBase<LeafContainerT,BranchContainerT>>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
        
}

void defineOctreeOctreePointcloudChangedetectorClasses(py::module &sub_module) {
}