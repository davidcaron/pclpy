#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_base.h>

using namespace pcl::octree;


template<typename LeafContainerT = int,
        typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreeBase(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreeBase<LeafContainerT, BranchContainerT>;
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
    cls.def("depth_begin", &Class::depth_begin, "max_depth_arg"_a=0);
    cls.def("depth_end", &Class::depth_end);
    cls.def("breadth_begin", &Class::breadth_begin, "max_depth_arg"_a=0);
    cls.def("breadth_end", &Class::breadth_end);
    // Operators not implemented (operator=);
    cls.def("createLeaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::createLeaf), "idx_x_arg"_a, "idx_y_arg"_a, "idx_z_arg"_a);
    cls.def("findLeaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::findLeaf), "idx_x_arg"_a, "idx_y_arg"_a, "idx_z_arg"_a);
    cls.def("existLeaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::existLeaf, py::const_), "idx_x_arg"_a, "idx_y_arg"_a, "idx_z_arg"_a);
    cls.def("removeLeaf", py::overload_cast<unsigned int, unsigned int, unsigned int> (&Class::removeLeaf), "idx_x_arg"_a, "idx_y_arg"_a, "idx_z_arg"_a);
    cls.def("deleteTree", &Class::deleteTree);
    cls.def("serializeTree", py::overload_cast<std::vector<char> &> (&Class::serializeTree), "binary_tree_out_arg"_a);
    cls.def("serializeTree", py::overload_cast<std::vector<char> &, std::vector<LeafContainerT *> &> (&Class::serializeTree), "binary_tree_out_arg"_a, "leaf_container_vector_arg"_a);
    cls.def("serializeLeafs", &Class::serializeLeafs, "leaf_container_vector_arg"_a);
    cls.def("deserializeTree", py::overload_cast<std::vector<char> &> (&Class::deserializeTree), "binary_tree_input_arg"_a);
    cls.def("deserializeTree", py::overload_cast<std::vector<char> &, std::vector<LeafContainerT *> &> (&Class::deserializeTree), "binary_tree_input_arg"_a, "leaf_container_vector_arg"_a);
    cls.def("setMaxVoxelIndex", &Class::setMaxVoxelIndex, "max_voxel_index_arg"_a);
    cls.def("setTreeDepth", &Class::setTreeDepth, "max_depth_arg"_a);
    cls.def("getTreeDepth", &Class::getTreeDepth);
    cls.def("getLeafCount", &Class::getLeafCount);
    cls.def("getBranchCount", &Class::getBranchCount);
        
}

void defineOctreeOctreeBaseFunctions(py::module &m) {
}

void defineOctreeOctreeBaseClasses(py::module &sub_module) {
    py::module sub_module_OctreeBase = sub_module.def_submodule("OctreeBase", "Submodule OctreeBase");
    defineOctreeOctreeBase<int, pcl::octree::OctreeContainerEmpty>(sub_module_OctreeBase, "int_octree::OctreeContainerEmpty");
    defineOctreeOctreeBase<pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeContainerEmpty>(sub_module_OctreeBase, "octree::OctreeContainerEmpty_octree::OctreeContainerEmpty");
    defineOctreeOctreeBase<pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty>(sub_module_OctreeBase, "octree::OctreeContainerPointIndices_octree::OctreeContainerEmpty");
    defineOctreeOctreeBase<pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreeBase, "octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZ>_octree::OctreeContainerEmpty");
    defineOctreeOctreeBase<pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreeBase, "octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZI>_octree::OctreeContainerEmpty");
    defineOctreeOctreeBase<pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreeBase, "octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZL>_octree::OctreeContainerEmpty");
    defineOctreeOctreeBase<pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreeBase, "octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGB>_octree::OctreeContainerEmpty");
    defineOctreeOctreeBase<pcl::octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>, pcl::octree::OctreeContainerEmpty>(sub_module_OctreeBase, "octree::OctreePointCloudVoxelCentroidContainer<pcl::PointXYZRGBA>_octree::OctreeContainerEmpty");
}