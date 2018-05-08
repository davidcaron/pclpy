
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

#include <pcl/search/search.h>

using namespace pcl::search;


template<typename PointT>
void defineSearchSearch(py::module &m, std::string const & suffix) {
    using Class = pcl::search::Search<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("nearestKSearch", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "point"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<const PointCloud &, int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "cloud"_a, "index"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "index"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<const PointCloud &, const std::vector<int> &, int, std::vector< std::vector<int> > &, std::vector< std::vector<float> > &> (&Class::nearestKSearch, py::const_), "cloud"_a, "indices"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("radiusSearch", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "point"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<const PointCloud &, int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "cloud"_a, "index"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "index"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<const PointCloud &, const std::vector<int> &, double, std::vector< std::vector<int> > &, std::vector< std::vector<float> > &, unsigned int> (&Class::radiusSearch, py::const_), "cloud"_a, "indices"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("setSortedResults", &Class::setSortedResults, "sorted"_a);
    cls.def("setInputCloud", py::overload_cast<const PointCloudConstPtr &, const IndicesConstPtr &> (&Class::setInputCloud), "cloud"_a, "indices"_a=pcl::IndicesConstPtr());
    cls.def("getName", &Class::getName);
    cls.def("getSortedResults", &Class::getSortedResults);
    cls.def("getInputCloud", &Class::getInputCloud);
    cls.def("getIndices", &Class::getIndices);
    cls.def("nearestKSearchT", py::overload_cast<const pcl::PointXYZ &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearchT<pcl::PointXYZ>, py::const_), "point"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearchT", py::overload_cast<const pcl::PointCloud<pcl::PointXYZ> &, const std::vector<int> &, int, std::vector< std::vector<int> > &, std::vector< std::vector<float> > &> (&Class::nearestKSearchT<pcl::PointXYZ>, py::const_), "cloud"_a, "indices"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("radiusSearchT", py::overload_cast<const pcl::PointXYZ &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearchT<pcl::PointXYZ>, py::const_), "point"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearchT", py::overload_cast<const pcl::PointCloud<pcl::PointXYZ> &, const std::vector<int> &, double, std::vector< std::vector<int> > &, std::vector< std::vector<float> > &, unsigned int> (&Class::radiusSearchT<pcl::PointXYZ>, py::const_), "cloud"_a, "indices"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
        
}

void defineSearchSearchFunctions(py::module &m) {
}

void defineSearchSearchClasses(py::module &sub_module) {
    py::module sub_module_Search = sub_module.def_submodule("Search", "Submodule Search");
    defineSearchSearch<pcl::Axis>(sub_module_Search, "Axis");
    defineSearchSearch<pcl::BRISKSignature512>(sub_module_Search, "BRISKSignature512");
    defineSearchSearch<pcl::Boundary>(sub_module_Search, "Boundary");
    defineSearchSearch<pcl::CPPFSignature>(sub_module_Search, "CPPFSignature");
    defineSearchSearch<pcl::ESFSignature640>(sub_module_Search, "ESFSignature640");
    defineSearchSearch<pcl::FPFHSignature33>(sub_module_Search, "FPFHSignature33");
    defineSearchSearch<pcl::GRSDSignature21>(sub_module_Search, "GRSDSignature21");
    defineSearchSearch<pcl::IntensityGradient>(sub_module_Search, "IntensityGradient");
    defineSearchSearch<pcl::InterestPoint>(sub_module_Search, "InterestPoint");
    defineSearchSearch<pcl::Label>(sub_module_Search, "Label");
    defineSearchSearch<pcl::MomentInvariants>(sub_module_Search, "MomentInvariants");
    defineSearchSearch<pcl::Narf36>(sub_module_Search, "Narf36");
    defineSearchSearch<pcl::Normal>(sub_module_Search, "Normal");
    defineSearchSearch<pcl::NormalBasedSignature12>(sub_module_Search, "NormalBasedSignature12");
    defineSearchSearch<pcl::PFHRGBSignature250>(sub_module_Search, "PFHRGBSignature250");
    defineSearchSearch<pcl::PFHSignature125>(sub_module_Search, "PFHSignature125");
    defineSearchSearch<pcl::PPFRGBSignature>(sub_module_Search, "PPFRGBSignature");
    defineSearchSearch<pcl::PPFSignature>(sub_module_Search, "PPFSignature");
    defineSearchSearch<pcl::PointDEM>(sub_module_Search, "PointDEM");
    defineSearchSearch<pcl::PointNormal>(sub_module_Search, "PointNormal");
    defineSearchSearch<pcl::PointSurfel>(sub_module_Search, "PointSurfel");
    defineSearchSearch<pcl::PointUV>(sub_module_Search, "PointUV");
    defineSearchSearch<pcl::PointWithRange>(sub_module_Search, "PointWithRange");
    defineSearchSearch<pcl::PointWithScale>(sub_module_Search, "PointWithScale");
    defineSearchSearch<pcl::PointWithViewpoint>(sub_module_Search, "PointWithViewpoint");
    defineSearchSearch<pcl::PointXY>(sub_module_Search, "PointXY");
    defineSearchSearch<pcl::PointXYZ>(sub_module_Search, "PointXYZ");
    defineSearchSearch<pcl::PointXYZHSV>(sub_module_Search, "PointXYZHSV");
    defineSearchSearch<pcl::PointXYZI>(sub_module_Search, "PointXYZI");
    defineSearchSearch<pcl::PointXYZINormal>(sub_module_Search, "PointXYZINormal");
    defineSearchSearch<pcl::PointXYZL>(sub_module_Search, "PointXYZL");
    defineSearchSearch<pcl::PointXYZLNormal>(sub_module_Search, "PointXYZLNormal");
    defineSearchSearch<pcl::PointXYZRGB>(sub_module_Search, "PointXYZRGB");
    defineSearchSearch<pcl::PointXYZRGBA>(sub_module_Search, "PointXYZRGBA");
    defineSearchSearch<pcl::PointXYZRGBL>(sub_module_Search, "PointXYZRGBL");
    defineSearchSearch<pcl::PointXYZRGBNormal>(sub_module_Search, "PointXYZRGBNormal");
    defineSearchSearch<pcl::PrincipalCurvatures>(sub_module_Search, "PrincipalCurvatures");
    defineSearchSearch<pcl::PrincipalRadiiRSD>(sub_module_Search, "PrincipalRadiiRSD");
    defineSearchSearch<pcl::ReferenceFrame>(sub_module_Search, "ReferenceFrame");
    defineSearchSearch<pcl::SHOT1344>(sub_module_Search, "SHOT1344");
    defineSearchSearch<pcl::SHOT352>(sub_module_Search, "SHOT352");
    defineSearchSearch<pcl::ShapeContext1980>(sub_module_Search, "ShapeContext1980");
    defineSearchSearch<pcl::UniqueShapeContext1960>(sub_module_Search, "UniqueShapeContext1960");
    defineSearchSearch<pcl::VFHSignature308>(sub_module_Search, "VFHSignature308");
}