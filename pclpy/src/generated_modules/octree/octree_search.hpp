
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_search.h>

using namespace pcl::octree;


template<typename PointT, typename LeafContainerT = OctreeContainerPointIndices ,  typename BranchContainerT = OctreeContainerEmpty >
void defineOctreeOctreePointCloudSearch(py::module &m, std::string const & suffix) {
    using Class = octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>;
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
    py::class_<Class, OctreePointCloud<PointT,LeafContainerT,BranchContainerT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution"_a);
    cls.def("voxel_search", py::overload_cast<const PointT &, std::vector<int> &> (&Class::voxelSearch));
    cls.def("voxel_search", py::overload_cast<const int, std::vector<int> &> (&Class::voxelSearch));
    cls.def("nearest_k_search", py::overload_cast<const PointCloud &, int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch));
    cls.def("nearest_k_search", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch));
    cls.def("nearest_k_search", py::overload_cast<int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch));
    cls.def("approx_nearest_search", py::overload_cast<const PointCloud &, int, int &, float &> (&Class::approxNearestSearch));
    cls.def("approx_nearest_search", py::overload_cast<const PointT &, int &, float &> (&Class::approxNearestSearch));
    cls.def("approx_nearest_search", py::overload_cast<int, int &, float &> (&Class::approxNearestSearch));
    cls.def("radius_search", py::overload_cast<const PointCloud &, int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch));
    cls.def("radius_search", py::overload_cast<const PointT &, const double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("radius_search", py::overload_cast<int, const double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("box_search", &Class::boxSearch);
        
}

void defineOctreeOctreeSearchClasses(py::module &sub_module) {
    py::module sub_module_OctreePointCloudSearch = sub_module.def_submodule("OctreePointCloudSearch", "Submodule OctreePointCloudSearch");
    defineOctreeOctreePointCloudSearch<InterestPoint>(sub_module_OctreePointCloudSearch, "InterestPoint");
    defineOctreeOctreePointCloudSearch<PointDEM>(sub_module_OctreePointCloudSearch, "PointDEM");
    defineOctreeOctreePointCloudSearch<PointNormal>(sub_module_OctreePointCloudSearch, "PointNormal");
    defineOctreeOctreePointCloudSearch<PointSurfel>(sub_module_OctreePointCloudSearch, "PointSurfel");
    defineOctreeOctreePointCloudSearch<PointWithRange>(sub_module_OctreePointCloudSearch, "PointWithRange");
    defineOctreeOctreePointCloudSearch<PointWithScale>(sub_module_OctreePointCloudSearch, "PointWithScale");
    defineOctreeOctreePointCloudSearch<PointWithViewpoint>(sub_module_OctreePointCloudSearch, "PointWithViewpoint");
    defineOctreeOctreePointCloudSearch<PointXYZ>(sub_module_OctreePointCloudSearch, "PointXYZ");
    defineOctreeOctreePointCloudSearch<PointXYZHSV>(sub_module_OctreePointCloudSearch, "PointXYZHSV");
    defineOctreeOctreePointCloudSearch<PointXYZI>(sub_module_OctreePointCloudSearch, "PointXYZI");
    defineOctreeOctreePointCloudSearch<PointXYZINormal>(sub_module_OctreePointCloudSearch, "PointXYZINormal");
    defineOctreeOctreePointCloudSearch<PointXYZL>(sub_module_OctreePointCloudSearch, "PointXYZL");
    defineOctreeOctreePointCloudSearch<PointXYZLNormal>(sub_module_OctreePointCloudSearch, "PointXYZLNormal");
    defineOctreeOctreePointCloudSearch<PointXYZRGB>(sub_module_OctreePointCloudSearch, "PointXYZRGB");
    defineOctreeOctreePointCloudSearch<PointXYZRGBA>(sub_module_OctreePointCloudSearch, "PointXYZRGBA");
    defineOctreeOctreePointCloudSearch<PointXYZRGBL>(sub_module_OctreePointCloudSearch, "PointXYZRGBL");
    defineOctreeOctreePointCloudSearch<PointXYZRGBNormal>(sub_module_OctreePointCloudSearch, "PointXYZRGBNormal");
}