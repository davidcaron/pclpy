
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/octree/octree2buf_base.h>

using namespace pcl::octree;


template<typename LeafContainerT = int,
             typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctree2BufBase(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::Octree2BufBase<LeafContainerT, BranchContainerT>;
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
    cls.def("begin", &Class::begin, "max_depth_arg"_a=0);
    cls.def("end", &Class::end);
    cls.def("leaf_begin", &Class::leaf_begin, "max_depth_arg"_a=0);
    cls.def("leaf_end", &Class::leaf_end);
    cls.def("depth_begin", &Class::depth_begin, "maxDepth_arg"_a=0);
    cls.def("depth_end", &Class::depth_end);
    cls.def("breadth_begin", &Class::breadth_begin, "max_depth_arg"_a=0);
    cls.def("breadth_end", &Class::breadth_end);
    // Operators not implemented (operator=);
    cls.def("createLeaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::createLeaf), "idx_x_arg"_a, "idx_y_arg"_a, "idx_z_arg"_a);
    cls.def("findLeaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::findLeaf), "idx_x_arg"_a, "idx_y_arg"_a, "idx_z_arg"_a);
    cls.def("existLeaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::existLeaf, py::const_), "idx_x_arg"_a, "idx_y_arg"_a, "idx_z_arg"_a);
    cls.def("removeLeaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::removeLeaf), "idx_x_arg"_a, "idx_y_arg"_a, "idx_z_arg"_a);
    cls.def("deleteTree", &Class::deleteTree);
    cls.def("deletePreviousBuffer", &Class::deletePreviousBuffer);
    cls.def("deleteCurrentBuffer", &Class::deleteCurrentBuffer);
    cls.def("switchBuffers", &Class::switchBuffers);
    cls.def("serializeTree", py::overload_cast<std::vector<char> &, bool> (&Class::serializeTree), "binary_tree_out_arg"_a, "do_XOR_encoding_arg"_a=false);
    cls.def("serializeTree", py::overload_cast<std::vector<char> &, std::vector<LeafContainerT *> &, bool> (&Class::serializeTree), "binary_tree_out_arg"_a, "leaf_container_vector_arg"_a, "do_XOR_encoding_arg"_a=false);
    cls.def("serializeLeafs", &Class::serializeLeafs, "leaf_container_vector_arg"_a);
    cls.def("serializeNewLeafs", &Class::serializeNewLeafs, "leaf_container_vector_arg"_a);
    cls.def("deserializeTree", py::overload_cast<std::vector<char> &, bool> (&Class::deserializeTree), "binary_tree_in_arg"_a, "do_XOR_decoding_arg"_a=false);
    cls.def("deserializeTree", py::overload_cast<std::vector<char> &, std::vector<LeafContainerT *> &, bool> (&Class::deserializeTree), "binary_tree_in_arg"_a, "leaf_container_vector_arg"_a, "do_XOR_decoding_arg"_a=false);
    cls.def("setMaxVoxelIndex", &Class::setMaxVoxelIndex, "max_voxel_index_arg"_a);
    cls.def("setTreeDepth", &Class::setTreeDepth, "depth_arg"_a);
    cls.def("getTreeDepth", &Class::getTreeDepth);
    cls.def("getLeafCount", &Class::getLeafCount);
    cls.def("getBranchCount", &Class::getBranchCount);
        
}

template<typename ContainerT>
void defineOctreeBufferedBranchNode(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::BufferedBranchNode<ContainerT>;
    py::class_<Class, pcl::octree::OctreeNode, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    // Operators not implemented (operator=);
    cls.def("deepCopy", &Class::deepCopy);
    cls.def("hasChild", &Class::hasChild, "buffer_arg"_a, "index_arg"_a);
    cls.def("reset", &Class::reset);
    // Operators not implemented (operator->);
    // Operators not implemented (operator->);
    // Operators not implemented (operator*);
    // Operators not implemented (operator*);
    cls.def("setChildPtr", &Class::setChildPtr, "buffer_arg"_a, "index_arg"_a, "newNode_arg"_a);
    cls.def("getChildPtr", &Class::getChildPtr, "buffer_arg"_a, "index_arg"_a);
    cls.def("getNodeType", &Class::getNodeType);
    cls.def("getContainer", py::overload_cast<> (&Class::getContainer, py::const_));
    cls.def("getContainer", py::overload_cast<> (&Class::getContainer));
    cls.def("getContainerPtr", py::overload_cast<> (&Class::getContainerPtr, py::const_));
    cls.def("getContainerPtr", py::overload_cast<> (&Class::getContainerPtr));
        
}

void defineOctreeOctree2bufBaseFunctions(py::module &m) {
}

void defineOctreeOctree2bufBaseClasses(py::module &sub_module) {
    py::module sub_module_Octree2BufBase = sub_module.def_submodule("Octree2BufBase", "Submodule Octree2BufBase");
    defineOctreeOctree2BufBase<int, pcl::octree::OctreeContainerEmpty>(sub_module_Octree2BufBase, "int_octree::OctreeContainerEmpty");
    defineOctreeOctree2BufBase<pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty>(sub_module_Octree2BufBase, "octree::OctreeContainerPointIndices_octree::OctreeContainerEmpty");
}