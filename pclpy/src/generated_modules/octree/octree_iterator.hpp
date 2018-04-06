
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_iterator.h>

using namespace pcl::octree;


void defineOctreeIteratorState(py::module &m) {
    using Class = octree::IteratorState;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "IteratorState");
    cls.def_readonly("node_", &Class::node_);
    cls.def_readonly("key_", &Class::key_);
    cls.def_readonly("depth_", &Class::depth_);
}

template<typename OctreeT>
void defineOctreeOctreeIteratorBase(py::module &m, std::string const & suffix) {
    using Class = octree::OctreeIteratorBase<OctreeT>;
    using LeafNode = Class::LeafNode;
    using BranchNode = Class::BranchNode;
    using LeafContainer = Class::LeafContainer;
    using BranchContainer = Class::BranchContainer;
    py::class_<Class, std::iterator<std::forward_iterator_tag,constOctreeNode,void,constOctreeNode*,constOctreeNode&>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "max_depth_arg"_a=0);
    cls.def(py::init<OctreeT*, unsigned int>(), "octree_arg"_a, "max_depth_arg"_a=0);
    cls.def(py::init<pcl::octree::OctreeIteratorBase, unsigned int>(), "src"_a, "max_depth_arg"_a=0);
    // Operators not implemented (operator*);
    // Operators not implemented (operator=);
    // Operators not implemented (operator==);
    // Operators not implemented (operator!=);
    cls.def("reset", &Class::reset);
    cls.def("is_branch_node", &Class::isBranchNode);
    cls.def("is_leaf_node", &Class::isLeafNode);
    cls.def("get_leaf_container", py::overload_cast<> (&Class::getLeafContainer, py::const_));
    cls.def("get_leaf_container", py::overload_cast<> (&Class::getLeafContainer));
    cls.def("get_branch_container", py::overload_cast<> (&Class::getBranchContainer, py::const_));
    cls.def("get_branch_container", py::overload_cast<> (&Class::getBranchContainer));
        
}

template<typename OctreeT>
void defineOctreeOctreeBreadthFirstIterator(py::module &m, std::string const & suffix) {
    using Class = octree::OctreeBreadthFirstIterator<OctreeT>;
    py::class_<Class, OctreeIteratorBase<OctreeT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "max_depth_arg"_a=0);
    cls.def(py::init<OctreeT*, unsigned int>(), "octree_arg"_a, "max_depth_arg"_a=0);
    // Operators not implemented (operator++);
    // Operators not implemented (operator++);
    // Operators not implemented (operator=);
    cls.def("reset", &Class::reset);
        
}

template<typename OctreeT>
void defineOctreeOctreeDepthFirstIterator(py::module &m, std::string const & suffix) {
    using Class = octree::OctreeDepthFirstIterator<OctreeT>;
    using LeafNode = Class::LeafNode;
    using BranchNode = Class::BranchNode;
    py::class_<Class, OctreeIteratorBase<OctreeT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "max_depth_arg"_a=0);
    cls.def(py::init<OctreeT*, unsigned int>(), "octree_arg"_a, "max_depth_arg"_a=0);
    // Operators not implemented (operator++);
    // Operators not implemented (operator++);
    // Operators not implemented (operator=);
    cls.def("reset", &Class::reset);
    cls.def("skip_child_voxels", &Class::skipChildVoxels);
        
}

template<typename OctreeT>
void defineOctreeOctreeLeafNodeIterator(py::module &m, std::string const & suffix) {
    using Class = octree::OctreeLeafNodeIterator<OctreeT>;
    py::class_<Class, OctreeDepthFirstIterator<OctreeT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "max_depth_arg"_a=0);
    cls.def(py::init<OctreeT*, unsigned int>(), "octree_arg"_a, "max_depth_arg"_a=0);
    // Operators not implemented (operator++);
    // Operators not implemented (operator++);
    // Operators not implemented (operator*);
    cls.def("reset", &Class::reset);
        
}

void defineOctreeOctreeIteratorClasses(py::module &sub_module) {
    defineOctreeIteratorState(sub_module);
}