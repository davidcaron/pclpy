
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/filters/filter_indices.h>



template<typename PointT>
void defineFiltersFilterIndices(py::module &m, std::string const & suffix) {
    using Class = pcl::FilterIndices<PointT>;
    using PointCloud = Class::PointCloud;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("filter", py::overload_cast<PointCloud &> (&Class::filter), "output"_a);
    cls.def("filter", py::overload_cast<std::vector<int> &> (&Class::filter), "indices"_a);
    cls.def("setNegative", &Class::setNegative, "negative"_a);
    cls.def("setKeepOrganized", &Class::setKeepOrganized, "keep_organized"_a);
    cls.def("setUserFilterValue", &Class::setUserFilterValue, "value"_a);
    cls.def("getNegative", &Class::getNegative);
    cls.def("getKeepOrganized", &Class::getKeepOrganized);
        
}

template<typename PointT>
void defineFiltersFilterIndicesFunctions1(py::module &m) {
    m.def("removeNaNFromPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, std::vector<int> &> (&pcl::removeNaNFromPointCloud<PointT>), "cloud_in"_a, "index"_a);
}

void defineFiltersFilterIndicesFunctions(py::module &m) {
    defineFiltersFilterIndicesFunctions1<pcl::PointXYZ>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointXYZI>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointXYZL>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointXYZRGBA>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointXYZRGB>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointXYZRGBL>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointXYZHSV>(m);
    defineFiltersFilterIndicesFunctions1<pcl::InterestPoint>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointNormal>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointXYZRGBNormal>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointXYZINormal>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointXYZLNormal>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointWithRange>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointWithViewpoint>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointWithScale>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointSurfel>(m);
    defineFiltersFilterIndicesFunctions1<pcl::PointDEM>(m);
}

void defineFiltersFilterIndicesClasses(py::module &sub_module) {
    py::module sub_module_FilterIndices = sub_module.def_submodule("FilterIndices", "Submodule FilterIndices");
    defineFiltersFilterIndices<pcl::Axis>(sub_module_FilterIndices, "Axis");
    defineFiltersFilterIndices<pcl::BRISKSignature512>(sub_module_FilterIndices, "BRISKSignature512");
    defineFiltersFilterIndices<pcl::Boundary>(sub_module_FilterIndices, "Boundary");
    defineFiltersFilterIndices<pcl::CPPFSignature>(sub_module_FilterIndices, "CPPFSignature");
    defineFiltersFilterIndices<pcl::ESFSignature640>(sub_module_FilterIndices, "ESFSignature640");
    defineFiltersFilterIndices<pcl::FPFHSignature33>(sub_module_FilterIndices, "FPFHSignature33");
    defineFiltersFilterIndices<pcl::GRSDSignature21>(sub_module_FilterIndices, "GRSDSignature21");
    defineFiltersFilterIndices<pcl::IntensityGradient>(sub_module_FilterIndices, "IntensityGradient");
    defineFiltersFilterIndices<pcl::InterestPoint>(sub_module_FilterIndices, "InterestPoint");
    defineFiltersFilterIndices<pcl::Label>(sub_module_FilterIndices, "Label");
    defineFiltersFilterIndices<pcl::MomentInvariants>(sub_module_FilterIndices, "MomentInvariants");
    defineFiltersFilterIndices<pcl::Narf36>(sub_module_FilterIndices, "Narf36");
    defineFiltersFilterIndices<pcl::Normal>(sub_module_FilterIndices, "Normal");
    defineFiltersFilterIndices<pcl::NormalBasedSignature12>(sub_module_FilterIndices, "NormalBasedSignature12");
    defineFiltersFilterIndices<pcl::PFHRGBSignature250>(sub_module_FilterIndices, "PFHRGBSignature250");
    defineFiltersFilterIndices<pcl::PFHSignature125>(sub_module_FilterIndices, "PFHSignature125");
    defineFiltersFilterIndices<pcl::PPFRGBSignature>(sub_module_FilterIndices, "PPFRGBSignature");
    defineFiltersFilterIndices<pcl::PPFSignature>(sub_module_FilterIndices, "PPFSignature");
    defineFiltersFilterIndices<pcl::PointDEM>(sub_module_FilterIndices, "PointDEM");
    defineFiltersFilterIndices<pcl::PointNormal>(sub_module_FilterIndices, "PointNormal");
    defineFiltersFilterIndices<pcl::PointSurfel>(sub_module_FilterIndices, "PointSurfel");
    defineFiltersFilterIndices<pcl::PointUV>(sub_module_FilterIndices, "PointUV");
    defineFiltersFilterIndices<pcl::PointWithRange>(sub_module_FilterIndices, "PointWithRange");
    defineFiltersFilterIndices<pcl::PointWithScale>(sub_module_FilterIndices, "PointWithScale");
    defineFiltersFilterIndices<pcl::PointWithViewpoint>(sub_module_FilterIndices, "PointWithViewpoint");
    defineFiltersFilterIndices<pcl::PointXY>(sub_module_FilterIndices, "PointXY");
    defineFiltersFilterIndices<pcl::PointXYZ>(sub_module_FilterIndices, "PointXYZ");
    defineFiltersFilterIndices<pcl::PointXYZHSV>(sub_module_FilterIndices, "PointXYZHSV");
    defineFiltersFilterIndices<pcl::PointXYZI>(sub_module_FilterIndices, "PointXYZI");
    defineFiltersFilterIndices<pcl::PointXYZINormal>(sub_module_FilterIndices, "PointXYZINormal");
    defineFiltersFilterIndices<pcl::PointXYZL>(sub_module_FilterIndices, "PointXYZL");
    defineFiltersFilterIndices<pcl::PointXYZLNormal>(sub_module_FilterIndices, "PointXYZLNormal");
    defineFiltersFilterIndices<pcl::PointXYZRGB>(sub_module_FilterIndices, "PointXYZRGB");
    defineFiltersFilterIndices<pcl::PointXYZRGBA>(sub_module_FilterIndices, "PointXYZRGBA");
    defineFiltersFilterIndices<pcl::PointXYZRGBL>(sub_module_FilterIndices, "PointXYZRGBL");
    defineFiltersFilterIndices<pcl::PointXYZRGBNormal>(sub_module_FilterIndices, "PointXYZRGBNormal");
    defineFiltersFilterIndices<pcl::PrincipalCurvatures>(sub_module_FilterIndices, "PrincipalCurvatures");
    defineFiltersFilterIndices<pcl::PrincipalRadiiRSD>(sub_module_FilterIndices, "PrincipalRadiiRSD");
    defineFiltersFilterIndices<pcl::ReferenceFrame>(sub_module_FilterIndices, "ReferenceFrame");
    defineFiltersFilterIndices<pcl::SHOT1344>(sub_module_FilterIndices, "SHOT1344");
    defineFiltersFilterIndices<pcl::SHOT352>(sub_module_FilterIndices, "SHOT352");
    defineFiltersFilterIndices<pcl::ShapeContext1980>(sub_module_FilterIndices, "ShapeContext1980");
    defineFiltersFilterIndices<pcl::UniqueShapeContext1960>(sub_module_FilterIndices, "UniqueShapeContext1960");
    defineFiltersFilterIndices<pcl::VFHSignature308>(sub_module_FilterIndices, "VFHSignature308");
    defineFiltersFilterIndicesFunctions(sub_module);
}