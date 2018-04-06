
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_base.h>

using namespace pcl::octree;


template<typename LeafContainerT = int,
        typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreeBase(py::module &m, std::string const & suffix) {
    using Class = octree::OctreeBase<LeafContainerT, BranchContainerT>;
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
    cls.def("serialize_tree", py::overload_cast<std::vector<char> &> (&Class::serializeTree));
    cls.def("serialize_tree", py::overload_cast<std::vector<char> &, std::vector<LeafContainerT *> &> (&Class::serializeTree));
    cls.def("deserialize_tree", py::overload_cast<std::vector<char> &> (&Class::deserializeTree));
    cls.def("deserialize_tree", py::overload_cast<std::vector<char> &, std::vector<LeafContainerT *> &> (&Class::deserializeTree));
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
    cls.def("serialize_leafs", &Class::serializeLeafs);
        
}

void defineOctreeOctreeBaseClasses(py::module &sub_module) {
    py::module sub_module_OctreeBase = sub_module.def_submodule("OctreeBase", "Submodule OctreeBase");
    defineOctreeOctreeBase<int, OctreeContainerEmpty>(sub_module_OctreeBase, "int_OctreeContainerEmpty");
    defineOctreeOctreeBase<OctreeContainerEmpty, OctreeContainerEmpty>(sub_module_OctreeBase, "OctreeContainerEmpty_OctreeContainerEmpty");
    defineOctreeOctreeBase<OctreeContainerPointIndices, OctreeContainerEmpty>(sub_module_OctreeBase, "OctreeContainerPointIndices_OctreeContainerEmpty");
}