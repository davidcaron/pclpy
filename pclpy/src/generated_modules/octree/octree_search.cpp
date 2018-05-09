
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/octree/octree_search.h>

using namespace pcl::octree;


template<typename PointT, typename LeafContainerT = OctreeContainerPointIndices ,  typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreePointCloudSearch(py::module &m, std::string const & suffix) {
    using Class = pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using AlignedPointTVector = Class::AlignedPointTVector;
    using OctreeT = Class::OctreeT;
    using LeafNode = Class::LeafNode;
    using BranchNode = Class::BranchNode;
    py::class_<Class, pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution"_a);
    cls.def("voxelSearch", py::overload_cast<const PointT &, std::vector<int> &> (&Class::voxelSearch), "point"_a, "point_idx_data"_a);
    cls.def("voxelSearch", py::overload_cast<const int, std::vector<int> &> (&Class::voxelSearch), "index"_a, "point_idx_data"_a);
    cls.def("nearestKSearch", py::overload_cast<const PointCloud &, int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch), "cloud"_a, "index"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch), "p_q"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch), "index"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("approxNearestSearch", py::overload_cast<const PointCloud &, int, int &, float &> (&Class::approxNearestSearch), "cloud"_a, "query_index"_a, "result_index"_a, "sqr_distance"_a);
    cls.def("approxNearestSearch", py::overload_cast<const PointT &, int &, float &> (&Class::approxNearestSearch), "p_q"_a, "result_index"_a, "sqr_distance"_a);
    cls.def("approxNearestSearch", py::overload_cast<int, int &, float &> (&Class::approxNearestSearch), "query_index"_a, "result_index"_a, "sqr_distance"_a);
    cls.def("radiusSearch", py::overload_cast<const PointCloud &, int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch), "cloud"_a, "index"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<const PointT &, const double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "p_q"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<int, const double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "index"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("boxSearch", &Class::boxSearch, "min_pt"_a, "max_pt"_a, "k_indices"_a);
    cls.def("getIntersectedVoxelCenters", &Class::getIntersectedVoxelCenters, "origin"_a, "direction"_a, "voxel_center_list"_a, "max_voxel_count"_a=0);
    cls.def("getIntersectedVoxelIndices", &Class::getIntersectedVoxelIndices, "origin"_a, "direction"_a, "k_indices"_a, "max_voxel_count"_a=0);
        
}

void defineOctreeOctreeSearchFunctions(py::module &m) {
}

void defineOctreeOctreeSearchClasses(py::module &sub_module) {
    py::module sub_module_OctreePointCloudSearch = sub_module.def_submodule("OctreePointCloudSearch", "Submodule OctreePointCloudSearch");
    defineOctreeOctreePointCloudSearch<pcl::InterestPoint>(sub_module_OctreePointCloudSearch, "InterestPoint");
    defineOctreeOctreePointCloudSearch<pcl::PointDEM>(sub_module_OctreePointCloudSearch, "PointDEM");
    defineOctreeOctreePointCloudSearch<pcl::PointNormal>(sub_module_OctreePointCloudSearch, "PointNormal");
    defineOctreeOctreePointCloudSearch<pcl::PointSurfel>(sub_module_OctreePointCloudSearch, "PointSurfel");
    defineOctreeOctreePointCloudSearch<pcl::PointWithRange>(sub_module_OctreePointCloudSearch, "PointWithRange");
    defineOctreeOctreePointCloudSearch<pcl::PointWithScale>(sub_module_OctreePointCloudSearch, "PointWithScale");
    defineOctreeOctreePointCloudSearch<pcl::PointWithViewpoint>(sub_module_OctreePointCloudSearch, "PointWithViewpoint");
    defineOctreeOctreePointCloudSearch<pcl::PointXYZ>(sub_module_OctreePointCloudSearch, "PointXYZ");
    defineOctreeOctreePointCloudSearch<pcl::PointXYZHSV>(sub_module_OctreePointCloudSearch, "PointXYZHSV");
    defineOctreeOctreePointCloudSearch<pcl::PointXYZI>(sub_module_OctreePointCloudSearch, "PointXYZI");
    defineOctreeOctreePointCloudSearch<pcl::PointXYZINormal>(sub_module_OctreePointCloudSearch, "PointXYZINormal");
    defineOctreeOctreePointCloudSearch<pcl::PointXYZL>(sub_module_OctreePointCloudSearch, "PointXYZL");
    defineOctreeOctreePointCloudSearch<pcl::PointXYZLNormal>(sub_module_OctreePointCloudSearch, "PointXYZLNormal");
    defineOctreeOctreePointCloudSearch<pcl::PointXYZRGB>(sub_module_OctreePointCloudSearch, "PointXYZRGB");
    defineOctreeOctreePointCloudSearch<pcl::PointXYZRGBA>(sub_module_OctreePointCloudSearch, "PointXYZRGBA");
    defineOctreeOctreePointCloudSearch<pcl::PointXYZRGBL>(sub_module_OctreePointCloudSearch, "PointXYZRGBL");
    defineOctreeOctreePointCloudSearch<pcl::PointXYZRGBNormal>(sub_module_OctreePointCloudSearch, "PointXYZRGBNormal");
}