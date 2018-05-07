
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/octree/octree_pointcloud_changedetector.h>

using namespace pcl::octree;


template<typename PointT,
        typename LeafContainerT = OctreeContainerPointIndices,
        typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreePointCloudChangeDetector(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreePointCloudChangeDetector<PointT, LeafContainerT, BranchContainerT>;
    py::class_<Class, pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT, pcl::octree::Octree2BufBase<pcl::octree::LeafContainer>Octree2BufBase<LeafContainerT, pcl::octree::BranchContainerT>>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
    cls.def("getPointIndicesFromNewVoxels", &Class::getPointIndicesFromNewVoxels, "indicesVector_arg"_a, "minPointsPerLeaf_arg"_a=0);
        
}

void defineOctreeOctreePointcloudChangedetectorFunctions(py::module &m) {
}

void defineOctreeOctreePointcloudChangedetectorClasses(py::module &sub_module) {
}