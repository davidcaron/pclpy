
#include <pcl/octree/octree_iterator.h>

using namespace pcl::octree;


void defineOctreeIteratorState(py::module &m) {
    using Class = pcl::octree::IteratorState;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "IteratorState");
    cls.def_readwrite("node_", &Class::node_);
    cls.def_readwrite("key_", &Class::key_);
    cls.def_readwrite("depth_", &Class::depth_);
}

template<typename OctreeT>
void defineOctreeOctreeIteratorBase(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreeIteratorBase<OctreeT>;
    using LeafNode = Class::LeafNode;
    using BranchNode = Class::BranchNode;
    using LeafContainer = Class::LeafContainer;
    using BranchContainer = Class::BranchContainer;
    py::class_<Class, std::iterator<std::forward_iterator_tag, pcl::octree::constOctreeNode, pcl::octree::void, pcl::octree::constOctreeNode*, pcl::octree::constOctreeNode&>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "max_depth_arg"_a=0);
    cls.def(py::init<OctreeT*, unsigned int>(), "octree_arg"_a, "max_depth_arg"_a=0);
    cls.def(py::init<pcl::octree::OctreeIteratorBase, unsigned int>(), "src"_a, "max_depth_arg"_a=0);
    // Operators not implemented (operator=);
    // Operators not implemented (operator==);
    // Operators not implemented (operator!=);
    cls.def("reset", &Class::reset);
    cls.def("isBranchNode", &Class::isBranchNode);
    cls.def("isLeafNode", &Class::isLeafNode);
    // Operators not implemented (operator*);
    cls.def("getCurrentOctreeKey", &Class::getCurrentOctreeKey);
    cls.def("getCurrentOctreeDepth", &Class::getCurrentOctreeDepth);
    cls.def("getCurrentOctreeNode", &Class::getCurrentOctreeNode);
    cls.def("getNodeConfiguration", &Class::getNodeConfiguration);
    cls.def("getLeafContainer", py::overload_cast<> (&Class::getLeafContainer, py::const_));
    cls.def("getLeafContainer", py::overload_cast<> (&Class::getLeafContainer));
    cls.def("getBranchContainer", py::overload_cast<> (&Class::getBranchContainer, py::const_));
    cls.def("getBranchContainer", py::overload_cast<> (&Class::getBranchContainer));
    cls.def("getNodeID", &Class::getNodeID);
        
}

template<typename OctreeT>
void defineOctreeOctreeBreadthFirstIterator(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreeBreadthFirstIterator<OctreeT>;
    py::class_<Class, pcl::octree::OctreeIteratorBase<pcl::octree::OctreeT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "max_depth_arg"_a=0);
    cls.def(py::init<OctreeT*, unsigned int>(), "octree_arg"_a, "max_depth_arg"_a=0);
    // Operators not implemented (operator=);
    cls.def("reset", &Class::reset);
    // Operators not implemented (operator++);
    // Operators not implemented (operator++);
        
}

template<typename OctreeT>
void defineOctreeOctreeDepthFirstIterator(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreeDepthFirstIterator<OctreeT>;
    using LeafNode = Class::LeafNode;
    using BranchNode = Class::BranchNode;
    py::class_<Class, pcl::octree::OctreeIteratorBase<pcl::octree::OctreeT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "max_depth_arg"_a=0);
    cls.def(py::init<OctreeT*, unsigned int>(), "octree_arg"_a, "max_depth_arg"_a=0);
    // Operators not implemented (operator=);
    cls.def("reset", &Class::reset);
    // Operators not implemented (operator++);
    // Operators not implemented (operator++);
    cls.def("skipChildVoxels", &Class::skipChildVoxels);
        
}

template<typename OctreeT>
void defineOctreeOctreeLeafNodeIterator(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreeLeafNodeIterator<OctreeT>;
    py::class_<Class, pcl::octree::OctreeDepthFirstIterator<pcl::octree::OctreeT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "max_depth_arg"_a=0);
    cls.def(py::init<OctreeT*, unsigned int>(), "octree_arg"_a, "max_depth_arg"_a=0);
    cls.def("reset", &Class::reset);
    // Operators not implemented (operator++);
    // Operators not implemented (operator++);
    // Operators not implemented (operator*);
        
}

void defineOctreeOctreeIteratorFunctions(py::module &m) {
}

void defineOctreeOctreeIteratorClasses(py::module &sub_module) {
    defineOctreeIteratorState(sub_module);
}