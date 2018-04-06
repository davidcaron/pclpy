
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/search/brute_force.h>

using namespace pcl::search;


template<typename PointT>
void defineSearchBruteForce(py::module &m, std::string const & suffix) {
    using Class = search::BruteForce<PointT>;
    py::class_<Class, Search<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "sorted_results"_a=false);
    cls.def("nearest_k_search", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
        
}

void defineSearchBruteForceClasses(py::module &sub_module) {
    py::module sub_module_BruteForce = sub_module.def_submodule("BruteForce", "Submodule BruteForce");
    defineSearchBruteForce<InterestPoint>(sub_module_BruteForce, "InterestPoint");
    defineSearchBruteForce<PointDEM>(sub_module_BruteForce, "PointDEM");
    defineSearchBruteForce<PointNormal>(sub_module_BruteForce, "PointNormal");
    defineSearchBruteForce<PointSurfel>(sub_module_BruteForce, "PointSurfel");
    defineSearchBruteForce<PointWithRange>(sub_module_BruteForce, "PointWithRange");
    defineSearchBruteForce<PointWithScale>(sub_module_BruteForce, "PointWithScale");
    defineSearchBruteForce<PointWithViewpoint>(sub_module_BruteForce, "PointWithViewpoint");
    defineSearchBruteForce<PointXYZ>(sub_module_BruteForce, "PointXYZ");
    defineSearchBruteForce<PointXYZHSV>(sub_module_BruteForce, "PointXYZHSV");
    defineSearchBruteForce<PointXYZI>(sub_module_BruteForce, "PointXYZI");
    defineSearchBruteForce<PointXYZINormal>(sub_module_BruteForce, "PointXYZINormal");
    defineSearchBruteForce<PointXYZL>(sub_module_BruteForce, "PointXYZL");
    defineSearchBruteForce<PointXYZLNormal>(sub_module_BruteForce, "PointXYZLNormal");
    defineSearchBruteForce<PointXYZRGB>(sub_module_BruteForce, "PointXYZRGB");
    defineSearchBruteForce<PointXYZRGBA>(sub_module_BruteForce, "PointXYZRGBA");
    defineSearchBruteForce<PointXYZRGBL>(sub_module_BruteForce, "PointXYZRGBL");
    defineSearchBruteForce<PointXYZRGBNormal>(sub_module_BruteForce, "PointXYZRGBNormal");
}