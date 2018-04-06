
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/search/organized.h>

using namespace pcl::search;


template<typename PointT>
void defineSearchOrganizedNeighbor(py::module &m, std::string const & suffix) {
    using Class = search::OrganizedNeighbor<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, search::Search<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool, float, unsigned>(), "sorted_results"_a=false, "eps"_a=1e-4f, "pyramid_level"_a=5);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("radius_search", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("nearest_k_search", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("is_valid", &Class::isValid);
    cls.def("compute_camera_matrix", &Class::computeCameraMatrix);
    cls.def("estimate_projection_matrix", &Class::estimateProjectionMatrix);
    cls.def("project_point", &Class::projectPoint);
        
}

void defineSearchOrganizedClasses(py::module &sub_module) {
    py::module sub_module_OrganizedNeighbor = sub_module.def_submodule("OrganizedNeighbor", "Submodule OrganizedNeighbor");
    defineSearchOrganizedNeighbor<InterestPoint>(sub_module_OrganizedNeighbor, "InterestPoint");
    defineSearchOrganizedNeighbor<PointDEM>(sub_module_OrganizedNeighbor, "PointDEM");
    defineSearchOrganizedNeighbor<PointNormal>(sub_module_OrganizedNeighbor, "PointNormal");
    defineSearchOrganizedNeighbor<PointSurfel>(sub_module_OrganizedNeighbor, "PointSurfel");
    defineSearchOrganizedNeighbor<PointWithRange>(sub_module_OrganizedNeighbor, "PointWithRange");
    defineSearchOrganizedNeighbor<PointWithScale>(sub_module_OrganizedNeighbor, "PointWithScale");
    defineSearchOrganizedNeighbor<PointWithViewpoint>(sub_module_OrganizedNeighbor, "PointWithViewpoint");
    defineSearchOrganizedNeighbor<PointXYZ>(sub_module_OrganizedNeighbor, "PointXYZ");
    defineSearchOrganizedNeighbor<PointXYZHSV>(sub_module_OrganizedNeighbor, "PointXYZHSV");
    defineSearchOrganizedNeighbor<PointXYZI>(sub_module_OrganizedNeighbor, "PointXYZI");
    defineSearchOrganizedNeighbor<PointXYZINormal>(sub_module_OrganizedNeighbor, "PointXYZINormal");
    defineSearchOrganizedNeighbor<PointXYZL>(sub_module_OrganizedNeighbor, "PointXYZL");
    defineSearchOrganizedNeighbor<PointXYZLNormal>(sub_module_OrganizedNeighbor, "PointXYZLNormal");
    defineSearchOrganizedNeighbor<PointXYZRGB>(sub_module_OrganizedNeighbor, "PointXYZRGB");
    defineSearchOrganizedNeighbor<PointXYZRGBA>(sub_module_OrganizedNeighbor, "PointXYZRGBA");
    defineSearchOrganizedNeighbor<PointXYZRGBL>(sub_module_OrganizedNeighbor, "PointXYZRGBL");
    defineSearchOrganizedNeighbor<PointXYZRGBNormal>(sub_module_OrganizedNeighbor, "PointXYZRGBNormal");
}