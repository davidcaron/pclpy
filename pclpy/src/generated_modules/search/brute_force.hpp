
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/search/brute_force.h>

using namespace pcl::search;


template<typename PointT>
void defineSearchBruteForce(py::module &m, std::string const & suffix) {
    using Class = pcl::search::BruteForce<PointT>;
    py::class_<Class, pcl::search::Search<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "sorted_results"_a=false);
    cls.def("nearestKSearch", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "point"_a, "k"_a, "k_indices"_a, "k_distances"_a);
    cls.def("radiusSearch", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "point"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
        
}

void defineSearchBruteForceFunctions(py::module &m) {
}

void defineSearchBruteForceClasses(py::module &sub_module) {
    py::module sub_module_BruteForce = sub_module.def_submodule("BruteForce", "Submodule BruteForce");
    defineSearchBruteForce<pcl::InterestPoint>(sub_module_BruteForce, "InterestPoint");
    defineSearchBruteForce<pcl::PointDEM>(sub_module_BruteForce, "PointDEM");
    defineSearchBruteForce<pcl::PointNormal>(sub_module_BruteForce, "PointNormal");
    defineSearchBruteForce<pcl::PointSurfel>(sub_module_BruteForce, "PointSurfel");
    defineSearchBruteForce<pcl::PointWithRange>(sub_module_BruteForce, "PointWithRange");
    defineSearchBruteForce<pcl::PointWithScale>(sub_module_BruteForce, "PointWithScale");
    defineSearchBruteForce<pcl::PointWithViewpoint>(sub_module_BruteForce, "PointWithViewpoint");
    defineSearchBruteForce<pcl::PointXYZ>(sub_module_BruteForce, "PointXYZ");
    defineSearchBruteForce<pcl::PointXYZHSV>(sub_module_BruteForce, "PointXYZHSV");
    defineSearchBruteForce<pcl::PointXYZI>(sub_module_BruteForce, "PointXYZI");
    defineSearchBruteForce<pcl::PointXYZINormal>(sub_module_BruteForce, "PointXYZINormal");
    defineSearchBruteForce<pcl::PointXYZL>(sub_module_BruteForce, "PointXYZL");
    defineSearchBruteForce<pcl::PointXYZLNormal>(sub_module_BruteForce, "PointXYZLNormal");
    defineSearchBruteForce<pcl::PointXYZRGB>(sub_module_BruteForce, "PointXYZRGB");
    defineSearchBruteForce<pcl::PointXYZRGBA>(sub_module_BruteForce, "PointXYZRGBA");
    defineSearchBruteForce<pcl::PointXYZRGBL>(sub_module_BruteForce, "PointXYZRGBL");
    defineSearchBruteForce<pcl::PointXYZRGBNormal>(sub_module_BruteForce, "PointXYZRGBNormal");
}