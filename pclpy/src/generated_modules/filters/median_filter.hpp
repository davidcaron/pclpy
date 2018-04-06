
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/median_filter.h>



template <typename PointT>
void defineFiltersMedianFilter(py::module &m, std::string const & suffix) {
    using Class = MedianFilter<PointT>;
    py::class_<Class, Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("window_size", &Class::getWindowSize, &Class::setWindowSize);
    cls.def_property("max_allowed_movement", &Class::getMaxAllowedMovement, &Class::setMaxAllowedMovement);
    cls.def("apply_filter", py::overload_cast<Filter<PointT>::PointCloud &> (&Class::applyFilter));
        
}

void defineFiltersMedianFilterClasses(py::module &sub_module) {
    py::module sub_module_MedianFilter = sub_module.def_submodule("MedianFilter", "Submodule MedianFilter");
    defineFiltersMedianFilter<InterestPoint>(sub_module_MedianFilter, "InterestPoint");
    defineFiltersMedianFilter<PointDEM>(sub_module_MedianFilter, "PointDEM");
    defineFiltersMedianFilter<PointNormal>(sub_module_MedianFilter, "PointNormal");
    defineFiltersMedianFilter<PointSurfel>(sub_module_MedianFilter, "PointSurfel");
    defineFiltersMedianFilter<PointWithRange>(sub_module_MedianFilter, "PointWithRange");
    defineFiltersMedianFilter<PointWithScale>(sub_module_MedianFilter, "PointWithScale");
    defineFiltersMedianFilter<PointWithViewpoint>(sub_module_MedianFilter, "PointWithViewpoint");
    defineFiltersMedianFilter<PointXYZ>(sub_module_MedianFilter, "PointXYZ");
    defineFiltersMedianFilter<PointXYZHSV>(sub_module_MedianFilter, "PointXYZHSV");
    defineFiltersMedianFilter<PointXYZI>(sub_module_MedianFilter, "PointXYZI");
    defineFiltersMedianFilter<PointXYZINormal>(sub_module_MedianFilter, "PointXYZINormal");
    defineFiltersMedianFilter<PointXYZL>(sub_module_MedianFilter, "PointXYZL");
    defineFiltersMedianFilter<PointXYZLNormal>(sub_module_MedianFilter, "PointXYZLNormal");
    defineFiltersMedianFilter<PointXYZRGB>(sub_module_MedianFilter, "PointXYZRGB");
    defineFiltersMedianFilter<PointXYZRGBA>(sub_module_MedianFilter, "PointXYZRGBA");
    defineFiltersMedianFilter<PointXYZRGBL>(sub_module_MedianFilter, "PointXYZRGBL");
    defineFiltersMedianFilter<PointXYZRGBNormal>(sub_module_MedianFilter, "PointXYZRGBNormal");
}