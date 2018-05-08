
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/search/organized.h>

using namespace pcl::search;


template<typename PointT>
void defineSearchOrganizedNeighbor(py::module &m, std::string const & suffix) {
    using Class = pcl::search::OrganizedNeighbor<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::search::Search<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool, float, unsigned>(), "sorted_results"_a=false, "eps"_a=1e-4f, "pyramid_level"_a=5);
    cls.def("isValid", &Class::isValid);
    cls.def("computeCameraMatrix", &Class::computeCameraMatrix, "camera_matrix"_a);
    cls.def("radiusSearch", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "p_q"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("estimateProjectionMatrix", &Class::estimateProjectionMatrix);
    cls.def("nearestKSearch", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "p_q"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("projectPoint", &Class::projectPoint, "p"_a, "q"_a);
    cls.def("setInputCloud", py::overload_cast<const PointCloudConstPtr &, const IndicesConstPtr &> (&Class::setInputCloud), "cloud"_a, "indices"_a=pcl::IndicesConstPtr());
        
}

void defineSearchOrganizedFunctions(py::module &m) {
}

void defineSearchOrganizedClasses(py::module &sub_module) {
    py::module sub_module_OrganizedNeighbor = sub_module.def_submodule("OrganizedNeighbor", "Submodule OrganizedNeighbor");
    defineSearchOrganizedNeighbor<pcl::InterestPoint>(sub_module_OrganizedNeighbor, "InterestPoint");
    defineSearchOrganizedNeighbor<pcl::PointDEM>(sub_module_OrganizedNeighbor, "PointDEM");
    defineSearchOrganizedNeighbor<pcl::PointNormal>(sub_module_OrganizedNeighbor, "PointNormal");
    defineSearchOrganizedNeighbor<pcl::PointSurfel>(sub_module_OrganizedNeighbor, "PointSurfel");
    defineSearchOrganizedNeighbor<pcl::PointWithRange>(sub_module_OrganizedNeighbor, "PointWithRange");
    defineSearchOrganizedNeighbor<pcl::PointWithScale>(sub_module_OrganizedNeighbor, "PointWithScale");
    defineSearchOrganizedNeighbor<pcl::PointWithViewpoint>(sub_module_OrganizedNeighbor, "PointWithViewpoint");
    defineSearchOrganizedNeighbor<pcl::PointXYZ>(sub_module_OrganizedNeighbor, "PointXYZ");
    defineSearchOrganizedNeighbor<pcl::PointXYZHSV>(sub_module_OrganizedNeighbor, "PointXYZHSV");
    defineSearchOrganizedNeighbor<pcl::PointXYZI>(sub_module_OrganizedNeighbor, "PointXYZI");
    defineSearchOrganizedNeighbor<pcl::PointXYZINormal>(sub_module_OrganizedNeighbor, "PointXYZINormal");
    defineSearchOrganizedNeighbor<pcl::PointXYZL>(sub_module_OrganizedNeighbor, "PointXYZL");
    defineSearchOrganizedNeighbor<pcl::PointXYZLNormal>(sub_module_OrganizedNeighbor, "PointXYZLNormal");
    defineSearchOrganizedNeighbor<pcl::PointXYZRGB>(sub_module_OrganizedNeighbor, "PointXYZRGB");
    defineSearchOrganizedNeighbor<pcl::PointXYZRGBA>(sub_module_OrganizedNeighbor, "PointXYZRGBA");
    defineSearchOrganizedNeighbor<pcl::PointXYZRGBL>(sub_module_OrganizedNeighbor, "PointXYZRGBL");
    defineSearchOrganizedNeighbor<pcl::PointXYZRGBNormal>(sub_module_OrganizedNeighbor, "PointXYZRGBNormal");
}