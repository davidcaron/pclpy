
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/octree/octree_nodes.h>

using namespace pcl::octree;


void defineOctreeOctreeNode(py::module &m) {
    using Class = pcl::octree::OctreeNode;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "OctreeNode");
    cls.def("deepCopy", &Class::deepCopy);
    cls.def("getNodeType", &Class::getNodeType);
}

template<typename ContainerT>
void defineOctreeOctreeBranchNode(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreeBranchNode<ContainerT>;
    py::class_<Class, pcl::octree::OctreeNode, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    // Operators not implemented (operator=);
    cls.def("deepCopy", &Class::deepCopy);
    // Operators not implemented (operator[]);
    cls.def("hasChild", &Class::hasChild, "child_idx_arg"_a);
    cls.def("reset", &Class::reset);
    // Operators not implemented (operator->);
    // Operators not implemented (operator->);
    // Operators not implemented (operator*);
    // Operators not implemented (operator*);
    cls.def("setChildPtr", &Class::setChildPtr, "child"_a, "index"_a);
    cls.def("getChildPtr", &Class::getChildPtr, "child_idx_arg"_a);
    cls.def("getNodeType", &Class::getNodeType);
    cls.def("getContainer", py::overload_cast<> (&Class::getContainer, py::const_));
    cls.def("getContainer", py::overload_cast<> (&Class::getContainer));
    cls.def("getContainerPtr", py::overload_cast<> (&Class::getContainerPtr, py::const_));
    cls.def("getContainerPtr", py::overload_cast<> (&Class::getContainerPtr));
        
}

template<typename ContainerT>
void defineOctreeOctreeLeafNode(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreeLeafNode<ContainerT>;
    py::class_<Class, pcl::octree::OctreeNode, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("deepCopy", &Class::deepCopy);
    // Operators not implemented (operator->);
    // Operators not implemented (operator->);
    // Operators not implemented (operator*);
    // Operators not implemented (operator*);
    cls.def("getNodeType", &Class::getNodeType);
    cls.def("getContainer", py::overload_cast<> (&Class::getContainer, py::const_));
    cls.def("getContainer", py::overload_cast<> (&Class::getContainer));
    cls.def("getContainerPtr", py::overload_cast<> (&Class::getContainerPtr, py::const_));
    cls.def("getContainerPtr", py::overload_cast<> (&Class::getContainerPtr));
        
}

void defineOctreeOctreeNodesFunctions(py::module &m) {
}

void defineOctreeOctreeNodesClasses(py::module &sub_module) {
    defineOctreeOctreeNode(sub_module);
    py::enum_<pcl::octree::node_type_t>(sub_module, "node_type_t")
        .value("BRANCH_NODE", pcl::octree::node_type_t::BRANCH_NODE)
        .value("LEAF_NODE", pcl::octree::node_type_t::LEAF_NODE)
        .export_values();
}