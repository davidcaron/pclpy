
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_pointcloud_density.h>

using namespace pcl::octree;


template<typename PointT, typename LeafContainerT = OctreePointCloudDensityContainer, typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreePointCloudDensity(py::module &m, std::string const & suffix) {
    using Class = octree::OctreePointCloudDensity<PointT, LeafContainerT, BranchContainerT>;
    py::class_<Class, OctreePointCloud<PointT,LeafContainerT,BranchContainerT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
        
}

void defineOctreeOctreePointCloudDensityContainer(py::module &m) {
    using Class = octree::OctreePointCloudDensityContainer;
    py::class_<Class, OctreeContainerBase, boost::shared_ptr<Class>> cls(m, "OctreePointCloudDensityContainer");
    cls.def(py::init<>());
    cls.def("deep_copy", &Class::deepCopy);
    // Operators not implemented (operator==);
    cls.def("add_point_index", &Class::addPointIndex);
    cls.def("reset", &Class::reset);
}

void defineOctreeOctreePointcloudDensityClasses(py::module &sub_module) {
    defineOctreeOctreePointCloudDensityContainer(sub_module);
}