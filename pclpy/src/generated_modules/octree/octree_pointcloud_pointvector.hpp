
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/octree/octree_pointcloud_pointvector.h>

using namespace pcl::octree;


template<typename PointT,
             typename LeafContainerT = OctreeContainerPointIndices,
             typename BranchContainerT = OctreeContainerEmpty,
             typename OctreeT = OctreeBase<LeafContainerT, BranchContainerT> >
void defineOctreeOctreePointCloudPointVector(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreePointCloudPointVector<PointT, LeafContainerT, BranchContainerT, OctreeT>;
    using SingleBuffer = Class::SingleBuffer;
    py::class_<Class, pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
        
}

void defineOctreeOctreePointcloudPointvectorFunctions(py::module &m) {
}

void defineOctreeOctreePointcloudPointvectorClasses(py::module &sub_module) {
}