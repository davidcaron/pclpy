
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/octree/octree_pointcloud_adjacency_container.h>

using namespace pcl::octree;


template<typename PointInT, typename DataT = PointInT>
void defineOctreeOctreePointCloudAdjacencyContainer(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreePointCloudAdjacencyContainer<PointInT, DataT>;
    using NeighborListT = Class::NeighborListT;
    using const_iterator = Class::const_iterator;
    py::class_<Class, pcl::octree::OctreeContainerBase, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("cbegin", &Class::cbegin);
    cls.def("cend", &Class::cend);
    cls.def("size", &Class::size);
    cls.def("setData", &Class::setData, "data_arg"_a);
    cls.def("getNumNeighbors", &Class::getNumNeighbors);
    cls.def("getPointCounter", &Class::getPointCounter);
    cls.def("getData", &Class::getData);
    cls.def("getSize", &Class::getSize);
        
}

void defineOctreeOctreePointcloudAdjacencyContainerFunctions(py::module &m) {
}

void defineOctreeOctreePointcloudAdjacencyContainerClasses(py::module &sub_module) {
}