
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/passthrough.h>



template <typename PointT>
void defineFiltersPassThrough(py::module &m, std::string const & suffix) {
    using Class = PassThrough<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def_property("filter_field_name", &Class::getFilterFieldName, &Class::setFilterFieldName);
    cls.def_property("filter_limits", &Class::getFilterLimits, &Class::setFilterLimits);
    cls.def("set_filter_limits_negative", &Class::setFilterLimitsNegative);
    cls.def("get_filter_limits_negative", py::overload_cast<bool &> (&Class::getFilterLimitsNegative));
    cls.def("get_filter_limits_negative", py::overload_cast<> (&Class::getFilterLimitsNegative));
        
}



void defineFiltersPassthroughClasses(py::module &sub_module) {
    py::module sub_module_PassThrough = sub_module.def_submodule("PassThrough", "Submodule PassThrough");
    defineFiltersPassThrough<InterestPoint>(sub_module_PassThrough, "InterestPoint");
    defineFiltersPassThrough<PointDEM>(sub_module_PassThrough, "PointDEM");
    defineFiltersPassThrough<PointNormal>(sub_module_PassThrough, "PointNormal");
    defineFiltersPassThrough<PointSurfel>(sub_module_PassThrough, "PointSurfel");
    defineFiltersPassThrough<PointWithRange>(sub_module_PassThrough, "PointWithRange");
    defineFiltersPassThrough<PointWithScale>(sub_module_PassThrough, "PointWithScale");
    defineFiltersPassThrough<PointWithViewpoint>(sub_module_PassThrough, "PointWithViewpoint");
    defineFiltersPassThrough<PointXYZ>(sub_module_PassThrough, "PointXYZ");
    defineFiltersPassThrough<PointXYZHSV>(sub_module_PassThrough, "PointXYZHSV");
    defineFiltersPassThrough<PointXYZI>(sub_module_PassThrough, "PointXYZI");
    defineFiltersPassThrough<PointXYZINormal>(sub_module_PassThrough, "PointXYZINormal");
    defineFiltersPassThrough<PointXYZL>(sub_module_PassThrough, "PointXYZL");
    defineFiltersPassThrough<PointXYZLNormal>(sub_module_PassThrough, "PointXYZLNormal");
    defineFiltersPassThrough<PointXYZRGB>(sub_module_PassThrough, "PointXYZRGB");
    defineFiltersPassThrough<PointXYZRGBA>(sub_module_PassThrough, "PointXYZRGBA");
    defineFiltersPassThrough<PointXYZRGBL>(sub_module_PassThrough, "PointXYZRGBL");
    defineFiltersPassThrough<PointXYZRGBNormal>(sub_module_PassThrough, "PointXYZRGBNormal");
}