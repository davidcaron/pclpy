
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

#include <pcl/filters/passthrough.h>



template <typename PointT>
void defineFiltersPassThrough(py::module &m, std::string const & suffix) {
    using Class = pcl::PassThrough<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def("setFilterFieldName", &Class::setFilterFieldName, "field_name"_a);
    cls.def("setFilterLimits", &Class::setFilterLimits, "limit_min"_a, "limit_max"_a);
    cls.def("setFilterLimitsNegative", &Class::setFilterLimitsNegative, "limit_negative"_a);
    cls.def("getFilterFieldName", &Class::getFilterFieldName);
    cls.def("getFilterLimits", &Class::getFilterLimits, "limit_min"_a, "limit_max"_a);
    cls.def("getFilterLimitsNegative", py::overload_cast<bool &> (&Class::getFilterLimitsNegative), "limit_negative"_a);
    cls.def("getFilterLimitsNegative", py::overload_cast<> (&Class::getFilterLimitsNegative));
        
}

void defineFiltersPassthroughFunctions(py::module &m) {
}

void defineFiltersPassthroughClasses(py::module &sub_module) {
    py::module sub_module_PassThrough = sub_module.def_submodule("PassThrough", "Submodule PassThrough");
    defineFiltersPassThrough<pcl::InterestPoint>(sub_module_PassThrough, "InterestPoint");
    defineFiltersPassThrough<pcl::PointDEM>(sub_module_PassThrough, "PointDEM");
    defineFiltersPassThrough<pcl::PointNormal>(sub_module_PassThrough, "PointNormal");
    defineFiltersPassThrough<pcl::PointSurfel>(sub_module_PassThrough, "PointSurfel");
    defineFiltersPassThrough<pcl::PointWithRange>(sub_module_PassThrough, "PointWithRange");
    defineFiltersPassThrough<pcl::PointWithScale>(sub_module_PassThrough, "PointWithScale");
    defineFiltersPassThrough<pcl::PointWithViewpoint>(sub_module_PassThrough, "PointWithViewpoint");
    defineFiltersPassThrough<pcl::PointXYZ>(sub_module_PassThrough, "PointXYZ");
    defineFiltersPassThrough<pcl::PointXYZHSV>(sub_module_PassThrough, "PointXYZHSV");
    defineFiltersPassThrough<pcl::PointXYZI>(sub_module_PassThrough, "PointXYZI");
    defineFiltersPassThrough<pcl::PointXYZINormal>(sub_module_PassThrough, "PointXYZINormal");
    defineFiltersPassThrough<pcl::PointXYZL>(sub_module_PassThrough, "PointXYZL");
    defineFiltersPassThrough<pcl::PointXYZLNormal>(sub_module_PassThrough, "PointXYZLNormal");
    defineFiltersPassThrough<pcl::PointXYZRGB>(sub_module_PassThrough, "PointXYZRGB");
    defineFiltersPassThrough<pcl::PointXYZRGBA>(sub_module_PassThrough, "PointXYZRGBA");
    defineFiltersPassThrough<pcl::PointXYZRGBL>(sub_module_PassThrough, "PointXYZRGBL");
    defineFiltersPassThrough<pcl::PointXYZRGBNormal>(sub_module_PassThrough, "PointXYZRGBNormal");
}