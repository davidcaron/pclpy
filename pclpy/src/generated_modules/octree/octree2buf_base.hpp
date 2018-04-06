
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree2buf_base.h>

using namespace pcl::octree;


template<typename LeafContainerT = int,
             typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctree2BufBase(py::module &m, std::string const & suffix) {
    using Class = octree::Octree2BufBase<LeafContainerT, BranchContainerT>;
    using OctreeT = Class::OctreeT;
    using BranchNode = Class::BranchNode;
    using LeafNode = Class::LeafNode;
    using BranchContainer = Class::BranchContainer;
    using LeafContainer = Class::LeafContainer;
    using Iterator = Class::Iterator;
    using ConstIterator = Class::ConstIterator;
    using LeafNodeIterator = Class::LeafNodeIterator;
    using ConstLeafNodeIterator = Class::ConstLeafNodeIterator;
    using DepthFirstIterator = Class::DepthFirstIterator;
    using ConstDepthFirstIterator = Class::ConstDepthFirstIterator;
    using BreadthFirstIterator = Class::BreadthFirstIterator;
    using ConstBreadthFirstIterator = Class::ConstBreadthFirstIterator;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_max_voxel_index", &Class::setMaxVoxelIndex);
    cls.def_property("tree_depth", &Class::getTreeDepth, &Class::setTreeDepth);
    cls.def("create_leaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::createLeaf));
    cls.def("find_leaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::findLeaf));
    cls.def("exist_leaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::existLeaf, py::const_));
    cls.def("remove_leaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::removeLeaf));
    cls.def("serialize_tree", py::overload_cast<std::vector<char> &, bool> (&Class::serializeTree));
    cls.def("serialize_tree", py::overload_cast<std::vector<char> &, std::vector<LeafContainerT *> &, bool> (&Class::serializeTree));
    cls.def("deserialize_tree", py::overload_cast<std::vector<char> &, bool> (&Class::deserializeTree));
    cls.def("deserialize_tree", py::overload_cast<std::vector<char> &, std::vector<LeafContainerT *> &, bool> (&Class::deserializeTree));
    cls.def("begin", &Class::begin);
    cls.def("end", &Class::end);
    cls.def("leaf_begin", &Class::leaf_begin);
    cls.def("leaf_end", &Class::leaf_end);
    cls.def("depth_begin", &Class::depth_begin);
    cls.def("depth_end", &Class::depth_end);
    cls.def("breadth_begin", &Class::breadth_begin);
    cls.def("breadth_end", &Class::breadth_end);
    // Operators not implemented (operator=);
    cls.def("delete_tree", &Class::deleteTree);
    cls.def("delete_previous_buffer", &Class::deletePreviousBuffer);
    cls.def("delete_current_buffer", &Class::deleteCurrentBuffer);
    cls.def("switch_buffers", &Class::switchBuffers);
    cls.def("serialize_leafs", &Class::serializeLeafs);
    cls.def("serialize_new_leafs", &Class::serializeNewLeafs);
        
}

template<typename ContainerT>
void defineOctreeBufferedBranchNode(py::module &m, std::string const & suffix) {
    using Class = octree::BufferedBranchNode<ContainerT>;
    py::class_<Class, OctreeNode, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("child_ptr", &Class::getChildPtr, &Class::setChildPtr);
    // Operators not implemented (operator->);
    // Operators not implemented (operator->);
    // Operators not implemented (operator*);
    // Operators not implemented (operator*);
    // Operators not implemented (operator=);
    cls.def("deep_copy", &Class::deepCopy);
    cls.def("has_child", &Class::hasChild);
    cls.def("reset", &Class::reset);
    cls.def("get_container", py::overload_cast<> (&Class::getContainer, py::const_));
    cls.def("get_container", py::overload_cast<> (&Class::getContainer));
    cls.def("get_container_ptr", py::overload_cast<> (&Class::getContainerPtr, py::const_));
    cls.def("get_container_ptr", py::overload_cast<> (&Class::getContainerPtr));
        
}

void defineOctreeOctree2bufBaseClasses(py::module &sub_module) {
    py::module sub_module_Octree2BufBase = sub_module.def_submodule("Octree2BufBase", "Submodule Octree2BufBase");
    defineOctreeOctree2BufBase<int, OctreeContainerEmpty>(sub_module_Octree2BufBase, "int_OctreeContainerEmpty");
    defineOctreeOctree2BufBase<OctreeContainerPointIndices, OctreeContainerEmpty>(sub_module_Octree2BufBase, "OctreeContainerPointIndices_OctreeContainerEmpty");
}