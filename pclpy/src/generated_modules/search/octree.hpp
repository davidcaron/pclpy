
#include <pcl/search/octree.h>

using namespace pcl::search;


template<typename PointT,
             typename LeafTWrap = pcl::octree::OctreeContainerPointIndices,
             typename BranchTWrap = pcl::octree::OctreeContainerEmpty,
             typename OctreeT = pcl::octree::OctreeBase<LeafTWrap, BranchTWrap > >
void defineSearchOctree(py::module &m, std::string const & suffix) {
    using Class = pcl::search::Octree<PointT, LeafTWrap, BranchTWrap, OctreeT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using OctreePointCloudSearchPtr = Class::OctreePointCloudSearchPtr;
    using OctreePointCloudSearchConstPtr = Class::OctreePointCloudSearchConstPtr;
    py::class_<Class, pcl::search::Search<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "resolution"_a);
    cls.def_readwrite("tree_", &Class::tree_);
    cls.def("nearestKSearch", py::overload_cast<const PointCloud &, int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "cloud"_a, "index"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "point"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "index"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("radiusSearch", py::overload_cast<const PointCloud &, int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "cloud"_a, "index"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "p_q"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "index"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("approxNearestSearch", py::overload_cast<const PointCloudConstPtr &, int, int &, float &> (&Class::approxNearestSearch), "cloud"_a, "query_index"_a, "result_index"_a, "sqr_distance"_a);
    cls.def("approxNearestSearch", py::overload_cast<const PointT &, int &, float &> (&Class::approxNearestSearch), "p_q"_a, "result_index"_a, "sqr_distance"_a);
    cls.def("approxNearestSearch", py::overload_cast<int, int &, float &> (&Class::approxNearestSearch), "query_index"_a, "result_index"_a, "sqr_distance"_a);
    cls.def("setInputCloud", py::overload_cast<const PointCloudConstPtr &> (&Class::setInputCloud), "cloud"_a);
    cls.def("setInputCloud", py::overload_cast<const PointCloudConstPtr &, const IndicesConstPtr &> (&Class::setInputCloud), "cloud"_a, "indices"_a);
        
}

void defineSearchOctreeFunctions(py::module &m) {
}

void defineSearchOctreeClasses(py::module &sub_module) {
    py::module sub_module_Octree = sub_module.def_submodule("Octree", "Submodule Octree");
    defineSearchOctree<pcl::InterestPoint>(sub_module_Octree, "InterestPoint");
    defineSearchOctree<pcl::PointDEM>(sub_module_Octree, "PointDEM");
    defineSearchOctree<pcl::PointNormal>(sub_module_Octree, "PointNormal");
    defineSearchOctree<pcl::PointSurfel>(sub_module_Octree, "PointSurfel");
    defineSearchOctree<pcl::PointWithRange>(sub_module_Octree, "PointWithRange");
    defineSearchOctree<pcl::PointWithScale>(sub_module_Octree, "PointWithScale");
    defineSearchOctree<pcl::PointWithViewpoint>(sub_module_Octree, "PointWithViewpoint");
    defineSearchOctree<pcl::PointXYZ>(sub_module_Octree, "PointXYZ");
    defineSearchOctree<pcl::PointXYZHSV>(sub_module_Octree, "PointXYZHSV");
    defineSearchOctree<pcl::PointXYZI>(sub_module_Octree, "PointXYZI");
    defineSearchOctree<pcl::PointXYZINormal>(sub_module_Octree, "PointXYZINormal");
    defineSearchOctree<pcl::PointXYZL>(sub_module_Octree, "PointXYZL");
    defineSearchOctree<pcl::PointXYZLNormal>(sub_module_Octree, "PointXYZLNormal");
    defineSearchOctree<pcl::PointXYZRGB>(sub_module_Octree, "PointXYZRGB");
    defineSearchOctree<pcl::PointXYZRGBA>(sub_module_Octree, "PointXYZRGBA");
    defineSearchOctree<pcl::PointXYZRGBL>(sub_module_Octree, "PointXYZRGBL");
    defineSearchOctree<pcl::PointXYZRGBNormal>(sub_module_Octree, "PointXYZRGBNormal");
}