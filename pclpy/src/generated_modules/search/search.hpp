
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/search/search.h>

using namespace pcl::search;


template<typename PointT>
void defineSearchSearch(py::module &m, std::string const & suffix) {
    using Class = search::Search<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("sorted_results", &Class::getSortedResults, &Class::setSortedResults);
    cls.def_property("input_cloud", &Class::getInputCloud, &Class::setInputCloud);
    cls.def("nearest_k_search", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("nearest_k_search", py::overload_cast<const PointCloud &, int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("nearest_k_search", py::overload_cast<int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("nearest_k_search", py::overload_cast<const PointCloud &, const std::vector<int> &, int, std::vector< std::vector<int> > &, std::vector< std::vector<float> > &> (&Class::nearestKSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointCloud &, int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("radius_search", py::overload_cast<int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointCloud &, const std::vector<int> &, double, std::vector< std::vector<int> > &, std::vector< std::vector<float> > &, unsigned int> (&Class::radiusSearch, py::const_));
        
}

void defineSearchSearchClasses(py::module &sub_module) {
    py::module sub_module_Search = sub_module.def_submodule("Search", "Submodule Search");
    defineSearchSearch<Axis>(sub_module_Search, "Axis");
    defineSearchSearch<BRISKSignature512>(sub_module_Search, "BRISKSignature512");
    defineSearchSearch<Boundary>(sub_module_Search, "Boundary");
    defineSearchSearch<CPPFSignature>(sub_module_Search, "CPPFSignature");
    defineSearchSearch<ESFSignature640>(sub_module_Search, "ESFSignature640");
    defineSearchSearch<FPFHSignature33>(sub_module_Search, "FPFHSignature33");
    defineSearchSearch<GRSDSignature21>(sub_module_Search, "GRSDSignature21");
    defineSearchSearch<IntensityGradient>(sub_module_Search, "IntensityGradient");
    defineSearchSearch<InterestPoint>(sub_module_Search, "InterestPoint");
    defineSearchSearch<Label>(sub_module_Search, "Label");
    defineSearchSearch<MomentInvariants>(sub_module_Search, "MomentInvariants");
    defineSearchSearch<Narf36>(sub_module_Search, "Narf36");
    defineSearchSearch<Normal>(sub_module_Search, "Normal");
    defineSearchSearch<NormalBasedSignature12>(sub_module_Search, "NormalBasedSignature12");
    defineSearchSearch<PFHRGBSignature250>(sub_module_Search, "PFHRGBSignature250");
    defineSearchSearch<PFHSignature125>(sub_module_Search, "PFHSignature125");
    defineSearchSearch<PPFRGBSignature>(sub_module_Search, "PPFRGBSignature");
    defineSearchSearch<PPFSignature>(sub_module_Search, "PPFSignature");
    defineSearchSearch<PointDEM>(sub_module_Search, "PointDEM");
    defineSearchSearch<PointNormal>(sub_module_Search, "PointNormal");
    defineSearchSearch<PointSurfel>(sub_module_Search, "PointSurfel");
    defineSearchSearch<PointUV>(sub_module_Search, "PointUV");
    defineSearchSearch<PointWithRange>(sub_module_Search, "PointWithRange");
    defineSearchSearch<PointWithScale>(sub_module_Search, "PointWithScale");
    defineSearchSearch<PointWithViewpoint>(sub_module_Search, "PointWithViewpoint");
    defineSearchSearch<PointXY>(sub_module_Search, "PointXY");
    defineSearchSearch<PointXYZ>(sub_module_Search, "PointXYZ");
    defineSearchSearch<PointXYZHSV>(sub_module_Search, "PointXYZHSV");
    defineSearchSearch<PointXYZI>(sub_module_Search, "PointXYZI");
    defineSearchSearch<PointXYZINormal>(sub_module_Search, "PointXYZINormal");
    defineSearchSearch<PointXYZL>(sub_module_Search, "PointXYZL");
    defineSearchSearch<PointXYZLNormal>(sub_module_Search, "PointXYZLNormal");
    defineSearchSearch<PointXYZRGB>(sub_module_Search, "PointXYZRGB");
    defineSearchSearch<PointXYZRGBA>(sub_module_Search, "PointXYZRGBA");
    defineSearchSearch<PointXYZRGBL>(sub_module_Search, "PointXYZRGBL");
    defineSearchSearch<PointXYZRGBNormal>(sub_module_Search, "PointXYZRGBNormal");
    defineSearchSearch<PrincipalCurvatures>(sub_module_Search, "PrincipalCurvatures");
    defineSearchSearch<PrincipalRadiiRSD>(sub_module_Search, "PrincipalRadiiRSD");
    defineSearchSearch<ReferenceFrame>(sub_module_Search, "ReferenceFrame");
    defineSearchSearch<SHOT1344>(sub_module_Search, "SHOT1344");
    defineSearchSearch<SHOT352>(sub_module_Search, "SHOT352");
    defineSearchSearch<ShapeContext1980>(sub_module_Search, "ShapeContext1980");
    defineSearchSearch<UniqueShapeContext1960>(sub_module_Search, "UniqueShapeContext1960");
    defineSearchSearch<VFHSignature308>(sub_module_Search, "VFHSignature308");
}