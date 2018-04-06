
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/search/octree.h>

using namespace pcl::search;


template<typename PointT,
             typename LeafTWrap = pcl::octree::OctreeContainerPointIndices,
             typename BranchTWrap = pcl::octree::OctreeContainerEmpty,
             typename OctreeT = pcl::octree::OctreeBase<LeafTWrap, BranchTWrap > >
void defineSearchOctree(py::module &m, std::string const & suffix) {
    using Class = search::Octree<PointT, LeafTWrap, BranchTWrap, OctreeT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using OctreePointCloudSearchPtr = Class::OctreePointCloudSearchPtr;
    using OctreePointCloudSearchConstPtr = Class::OctreePointCloudSearchConstPtr;
    py::class_<Class, Search<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution"_a);
    cls.def_readonly("tree_", &Class::tree_);
    cls.def("nearest_k_search", py::overload_cast<const PointCloud &, int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("nearest_k_search", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("nearest_k_search", py::overload_cast<int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointCloud &, int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("radius_search", py::overload_cast<int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("approx_nearest_search", py::overload_cast<const PointCloudConstPtr &, int, int &, float &> (&Class::approxNearestSearch));
    cls.def("approx_nearest_search", py::overload_cast<const PointT &, int &, float &> (&Class::approxNearestSearch));
    cls.def("approx_nearest_search", py::overload_cast<int, int &, float &> (&Class::approxNearestSearch));
    cls.def("set_input_cloud", py::overload_cast<const PointCloudConstPtr &> (&Class::setInputCloud));
    cls.def("set_input_cloud", py::overload_cast<const PointCloudConstPtr &, const IndicesConstPtr &> (&Class::setInputCloud));
        
}

void defineSearchOctreeClasses(py::module &sub_module) {
    py::module sub_module_Octree = sub_module.def_submodule("Octree", "Submodule Octree");
    defineSearchOctree<InterestPoint>(sub_module_Octree, "InterestPoint");
    defineSearchOctree<PointDEM>(sub_module_Octree, "PointDEM");
    defineSearchOctree<PointNormal>(sub_module_Octree, "PointNormal");
    defineSearchOctree<PointSurfel>(sub_module_Octree, "PointSurfel");
    defineSearchOctree<PointWithRange>(sub_module_Octree, "PointWithRange");
    defineSearchOctree<PointWithScale>(sub_module_Octree, "PointWithScale");
    defineSearchOctree<PointWithViewpoint>(sub_module_Octree, "PointWithViewpoint");
    defineSearchOctree<PointXYZ>(sub_module_Octree, "PointXYZ");
    defineSearchOctree<PointXYZHSV>(sub_module_Octree, "PointXYZHSV");
    defineSearchOctree<PointXYZI>(sub_module_Octree, "PointXYZI");
    defineSearchOctree<PointXYZINormal>(sub_module_Octree, "PointXYZINormal");
    defineSearchOctree<PointXYZL>(sub_module_Octree, "PointXYZL");
    defineSearchOctree<PointXYZLNormal>(sub_module_Octree, "PointXYZLNormal");
    defineSearchOctree<PointXYZRGB>(sub_module_Octree, "PointXYZRGB");
    defineSearchOctree<PointXYZRGBA>(sub_module_Octree, "PointXYZRGBA");
    defineSearchOctree<PointXYZRGBL>(sub_module_Octree, "PointXYZRGBL");
    defineSearchOctree<PointXYZRGBNormal>(sub_module_Octree, "PointXYZRGBNormal");
}