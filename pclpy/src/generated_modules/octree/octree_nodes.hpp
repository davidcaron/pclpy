
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_nodes.h>

using namespace pcl::octree;


void defineOctreeOctreeNode(py::module &m) {
    using Class = octree::OctreeNode;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "OctreeNode");
    cls.def("deep_copy", &Class::deepCopy);
}

template<typename ContainerT>
void defineOctreeOctreeBranchNode(py::module &m, std::string const & suffix) {
    using Class = octree::OctreeBranchNode<ContainerT>;
    py::class_<Class, OctreeNode, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("child_ptr", &Class::getChildPtr, &Class::setChildPtr);
    // Operators not implemented (operator->);
    // Operators not implemented (operator->);
    // Operators not implemented (operator*);
    // Operators not implemented (operator*);
    // Operators not implemented (operator=);
    cls.def("deep_copy", &Class::deepCopy);
    // Operators not implemented (operator[]);
    cls.def("has_child", &Class::hasChild);
    cls.def("reset", &Class::reset);
    cls.def("get_container", py::overload_cast<> (&Class::getContainer, py::const_));
    cls.def("get_container", py::overload_cast<> (&Class::getContainer));
    cls.def("get_container_ptr", py::overload_cast<> (&Class::getContainerPtr, py::const_));
    cls.def("get_container_ptr", py::overload_cast<> (&Class::getContainerPtr));
        
}

template<typename ContainerT>
void defineOctreeOctreeLeafNode(py::module &m, std::string const & suffix) {
    using Class = octree::OctreeLeafNode<ContainerT>;
    py::class_<Class, OctreeNode, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    // Operators not implemented (operator->);
    // Operators not implemented (operator->);
    // Operators not implemented (operator*);
    // Operators not implemented (operator*);
    cls.def("deep_copy", &Class::deepCopy);
    cls.def("get_container", py::overload_cast<> (&Class::getContainer, py::const_));
    cls.def("get_container", py::overload_cast<> (&Class::getContainer));
    cls.def("get_container_ptr", py::overload_cast<> (&Class::getContainerPtr, py::const_));
    cls.def("get_container_ptr", py::overload_cast<> (&Class::getContainerPtr));
        
}

void defineOctreeOctreeNodesClasses(py::module &sub_module) {
    defineOctreeOctreeNode(sub_module);
}