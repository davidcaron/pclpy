
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_pointcloud.h>

using namespace pcl::octree;


template<typename PointT, typename LeafContainerT = OctreeContainerPointIndices,
        typename BranchContainerT = OctreeContainerEmpty,
        typename OctreeT = OctreeBase<LeafContainerT, BranchContainerT> >
void defineOctreeOctreePointCloud(py::module &m, std::string const & suffix) {
    using Class = octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>;
    using Base = Class::Base;
    using LeafNode = Class::LeafNode;
    using BranchNode = Class::BranchNode;
    using Iterator = Class::Iterator;
    using ConstIterator = Class::ConstIterator;
    using LeafNodeIterator = Class::LeafNodeIterator;
    using ConstLeafNodeIterator = Class::ConstLeafNodeIterator;
    using DepthFirstIterator = Class::DepthFirstIterator;
    using ConstDepthFirstIterator = Class::ConstDepthFirstIterator;
    using BreadthFirstIterator = Class::BreadthFirstIterator;
    using ConstBreadthFirstIterator = Class::ConstBreadthFirstIterator;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using SingleBuffer = Class::SingleBuffer;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using AlignedPointTVector = Class::AlignedPointTVector;
    using AlignedPointXYZVector = Class::AlignedPointXYZVector;
    py::class_<Class, OctreeT, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution_arg"_a);
    cls.def_property("input_cloud", &Class::getInputCloud, &Class::setInputCloud);
    cls.def_property("epsilon", &Class::getEpsilon, &Class::setEpsilon);
    cls.def_property("resolution", &Class::getResolution, &Class::setResolution);
    cls.def("add_point_to_cloud", py::overload_cast<const PointT &, PointCloudPtr> (&Class::addPointToCloud));
    cls.def("add_point_to_cloud", py::overload_cast<const PointT &, PointCloudPtr, IndicesPtr> (&Class::addPointToCloud));
    cls.def("is_voxel_occupied_at_point", py::overload_cast<const PointT &> (&Class::isVoxelOccupiedAtPoint, py::const_));
    cls.def("is_voxel_occupied_at_point", py::overload_cast<const double, const double, const double> (&Class::isVoxelOccupiedAtPoint, py::const_));
    cls.def("is_voxel_occupied_at_point", py::overload_cast<const int &> (&Class::isVoxelOccupiedAtPoint, py::const_));
    cls.def("delete_voxel_at_point", py::overload_cast<const PointT &> (&Class::deleteVoxelAtPoint));
    cls.def("delete_voxel_at_point", py::overload_cast<const int &> (&Class::deleteVoxelAtPoint));
    cls.def("define_bounding_box", py::overload_cast<> (&Class::defineBoundingBox));
    cls.def("define_bounding_box", py::overload_cast<const double, const double, const double, const double, const double, const double> (&Class::defineBoundingBox));
    cls.def("define_bounding_box", py::overload_cast<const double, const double, const double> (&Class::defineBoundingBox));
    cls.def("define_bounding_box", py::overload_cast<const double> (&Class::defineBoundingBox));
    cls.def("add_points_from_input_cloud", &Class::addPointsFromInputCloud);
    cls.def("add_point_from_cloud", &Class::addPointFromCloud);
    cls.def("delete_tree", &Class::deleteTree);
    cls.def("enable_dynamic_depth", &Class::enableDynamicDepth);
    cls.def("get_voxel_squared_diameter", py::overload_cast<unsigned int> (&Class::getVoxelSquaredDiameter, py::const_));
    cls.def("get_voxel_squared_diameter", py::overload_cast<> (&Class::getVoxelSquaredDiameter, py::const_));
    cls.def("get_voxel_squared_side_len", py::overload_cast<unsigned int> (&Class::getVoxelSquaredSideLen, py::const_));
    cls.def("get_voxel_squared_side_len", py::overload_cast<> (&Class::getVoxelSquaredSideLen, py::const_));
        
}

void defineOctreeOctreePointcloudClasses(py::module &sub_module) {
    py::module sub_module_OctreePointCloud = sub_module.def_submodule("OctreePointCloud", "Submodule OctreePointCloud");
    defineOctreeOctreePointCloud<InterestPoint>(sub_module_OctreePointCloud, "InterestPoint");
    defineOctreeOctreePointCloud<PointDEM>(sub_module_OctreePointCloud, "PointDEM");
    defineOctreeOctreePointCloud<PointNormal>(sub_module_OctreePointCloud, "PointNormal");
    defineOctreeOctreePointCloud<PointSurfel>(sub_module_OctreePointCloud, "PointSurfel");
    defineOctreeOctreePointCloud<PointWithRange>(sub_module_OctreePointCloud, "PointWithRange");
    defineOctreeOctreePointCloud<PointWithScale>(sub_module_OctreePointCloud, "PointWithScale");
    defineOctreeOctreePointCloud<PointWithViewpoint>(sub_module_OctreePointCloud, "PointWithViewpoint");
    defineOctreeOctreePointCloud<PointXYZ>(sub_module_OctreePointCloud, "PointXYZ");
    defineOctreeOctreePointCloud<PointXYZHSV>(sub_module_OctreePointCloud, "PointXYZHSV");
    defineOctreeOctreePointCloud<PointXYZI>(sub_module_OctreePointCloud, "PointXYZI");
    defineOctreeOctreePointCloud<PointXYZINormal>(sub_module_OctreePointCloud, "PointXYZINormal");
    defineOctreeOctreePointCloud<PointXYZL>(sub_module_OctreePointCloud, "PointXYZL");
    defineOctreeOctreePointCloud<PointXYZLNormal>(sub_module_OctreePointCloud, "PointXYZLNormal");
    defineOctreeOctreePointCloud<PointXYZRGB>(sub_module_OctreePointCloud, "PointXYZRGB");
    defineOctreeOctreePointCloud<PointXYZRGBA>(sub_module_OctreePointCloud, "PointXYZRGBA");
    defineOctreeOctreePointCloud<PointXYZRGBL>(sub_module_OctreePointCloud, "PointXYZRGBL");
    defineOctreeOctreePointCloud<PointXYZRGBNormal>(sub_module_OctreePointCloud, "PointXYZRGBNormal");
}