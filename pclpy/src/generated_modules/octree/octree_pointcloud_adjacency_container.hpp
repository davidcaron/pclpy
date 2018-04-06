
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_pointcloud_adjacency_container.h>

using namespace pcl::octree;


template<typename PointInT, typename DataT = PointInT>
void defineOctreeOctreePointCloudAdjacencyContainer(py::module &m, std::string const & suffix) {
    using Class = octree::OctreePointCloudAdjacencyContainer<PointInT, DataT>;
    using NeighborListT = Class::NeighborListT;
    using const_iterator = Class::const_iterator;
    py::class_<Class, OctreeContainerBase, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("data", &Class::getData, &Class::setData);
    cls.def("cbegin", &Class::cbegin);
    cls.def("cend", &Class::cend);
    cls.def("size", &Class::size);
        
}

void defineOctreeOctreePointcloudAdjacencyContainerClasses(py::module &sub_module) {
}