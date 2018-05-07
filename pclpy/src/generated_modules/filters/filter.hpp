
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/filters/filter.h>



template<typename PointT>
void defineFiltersFilter(py::module &m, std::string const & suffix) {
    using Class = pcl::Filter<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("filter", py::overload_cast<PointCloud &> (&Class::filter), "output"_a);
    cls.def("getRemovedIndices", py::overload_cast<> (&Class::getRemovedIndices));
    cls.def("getRemovedIndices", py::overload_cast<pcl::PointIndices &> (&Class::getRemovedIndices), "pi"_a);
        
}

template<typename PointT>
void defineFiltersFilterFunctions1(py::module &m) {
    m.def("removeNaNFromPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, pcl::PointCloud<PointT> &, std::vector<int> &> (&pcl::removeNaNFromPointCloud<PointT>), "cloud_in"_a, "cloud_out"_a, "index"_a);
}

void defineFiltersFilterFunctions(py::module &m) {
    defineFiltersFilterFunctions1<pcl::PointXYZ>(m);
    defineFiltersFilterFunctions1<pcl::PointXYZI>(m);
    defineFiltersFilterFunctions1<pcl::PointXYZL>(m);
    defineFiltersFilterFunctions1<pcl::PointXYZRGBA>(m);
    defineFiltersFilterFunctions1<pcl::PointXYZRGB>(m);
    defineFiltersFilterFunctions1<pcl::PointXYZRGBL>(m);
    defineFiltersFilterFunctions1<pcl::PointXYZHSV>(m);
    defineFiltersFilterFunctions1<pcl::InterestPoint>(m);
    defineFiltersFilterFunctions1<pcl::PointNormal>(m);
    defineFiltersFilterFunctions1<pcl::PointXYZRGBNormal>(m);
    defineFiltersFilterFunctions1<pcl::PointXYZINormal>(m);
    defineFiltersFilterFunctions1<pcl::PointXYZLNormal>(m);
    defineFiltersFilterFunctions1<pcl::PointWithRange>(m);
    defineFiltersFilterFunctions1<pcl::PointWithViewpoint>(m);
    defineFiltersFilterFunctions1<pcl::PointWithScale>(m);
    defineFiltersFilterFunctions1<pcl::PointSurfel>(m);
    defineFiltersFilterFunctions1<pcl::PointDEM>(m);
}

void defineFiltersFilterClasses(py::module &sub_module) {
    py::module sub_module_Filter = sub_module.def_submodule("Filter", "Submodule Filter");
    defineFiltersFilter<pcl::Axis>(sub_module_Filter, "Axis");
    defineFiltersFilter<pcl::BRISKSignature512>(sub_module_Filter, "BRISKSignature512");
    defineFiltersFilter<pcl::Boundary>(sub_module_Filter, "Boundary");
    defineFiltersFilter<pcl::CPPFSignature>(sub_module_Filter, "CPPFSignature");
    defineFiltersFilter<pcl::ESFSignature640>(sub_module_Filter, "ESFSignature640");
    defineFiltersFilter<pcl::FPFHSignature33>(sub_module_Filter, "FPFHSignature33");
    defineFiltersFilter<pcl::GRSDSignature21>(sub_module_Filter, "GRSDSignature21");
    defineFiltersFilter<pcl::IntensityGradient>(sub_module_Filter, "IntensityGradient");
    defineFiltersFilter<pcl::InterestPoint>(sub_module_Filter, "InterestPoint");
    defineFiltersFilter<pcl::Label>(sub_module_Filter, "Label");
    defineFiltersFilter<pcl::MomentInvariants>(sub_module_Filter, "MomentInvariants");
    defineFiltersFilter<pcl::Narf36>(sub_module_Filter, "Narf36");
    defineFiltersFilter<pcl::Normal>(sub_module_Filter, "Normal");
    defineFiltersFilter<pcl::NormalBasedSignature12>(sub_module_Filter, "NormalBasedSignature12");
    defineFiltersFilter<pcl::PFHRGBSignature250>(sub_module_Filter, "PFHRGBSignature250");
    defineFiltersFilter<pcl::PFHSignature125>(sub_module_Filter, "PFHSignature125");
    defineFiltersFilter<pcl::PPFRGBSignature>(sub_module_Filter, "PPFRGBSignature");
    defineFiltersFilter<pcl::PPFSignature>(sub_module_Filter, "PPFSignature");
    defineFiltersFilter<pcl::PointDEM>(sub_module_Filter, "PointDEM");
    defineFiltersFilter<pcl::PointNormal>(sub_module_Filter, "PointNormal");
    defineFiltersFilter<pcl::PointSurfel>(sub_module_Filter, "PointSurfel");
    defineFiltersFilter<pcl::PointUV>(sub_module_Filter, "PointUV");
    defineFiltersFilter<pcl::PointWithRange>(sub_module_Filter, "PointWithRange");
    defineFiltersFilter<pcl::PointWithScale>(sub_module_Filter, "PointWithScale");
    defineFiltersFilter<pcl::PointWithViewpoint>(sub_module_Filter, "PointWithViewpoint");
    defineFiltersFilter<pcl::PointXY>(sub_module_Filter, "PointXY");
    defineFiltersFilter<pcl::PointXYZ>(sub_module_Filter, "PointXYZ");
    defineFiltersFilter<pcl::PointXYZHSV>(sub_module_Filter, "PointXYZHSV");
    defineFiltersFilter<pcl::PointXYZI>(sub_module_Filter, "PointXYZI");
    defineFiltersFilter<pcl::PointXYZINormal>(sub_module_Filter, "PointXYZINormal");
    defineFiltersFilter<pcl::PointXYZL>(sub_module_Filter, "PointXYZL");
    defineFiltersFilter<pcl::PointXYZLNormal>(sub_module_Filter, "PointXYZLNormal");
    defineFiltersFilter<pcl::PointXYZRGB>(sub_module_Filter, "PointXYZRGB");
    defineFiltersFilter<pcl::PointXYZRGBA>(sub_module_Filter, "PointXYZRGBA");
    defineFiltersFilter<pcl::PointXYZRGBL>(sub_module_Filter, "PointXYZRGBL");
    defineFiltersFilter<pcl::PointXYZRGBNormal>(sub_module_Filter, "PointXYZRGBNormal");
    defineFiltersFilter<pcl::PrincipalCurvatures>(sub_module_Filter, "PrincipalCurvatures");
    defineFiltersFilter<pcl::PrincipalRadiiRSD>(sub_module_Filter, "PrincipalRadiiRSD");
    defineFiltersFilter<pcl::ReferenceFrame>(sub_module_Filter, "ReferenceFrame");
    defineFiltersFilter<pcl::SHOT1344>(sub_module_Filter, "SHOT1344");
    defineFiltersFilter<pcl::SHOT352>(sub_module_Filter, "SHOT352");
    defineFiltersFilter<pcl::ShapeContext1980>(sub_module_Filter, "ShapeContext1980");
    defineFiltersFilter<pcl::UniqueShapeContext1960>(sub_module_Filter, "UniqueShapeContext1960");
    defineFiltersFilter<pcl::VFHSignature308>(sub_module_Filter, "VFHSignature308");
    defineFiltersFilterFunctions(sub_module);
}