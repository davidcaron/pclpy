
#include <pcl/filters/median_filter.h>



template <typename PointT>
void defineFiltersMedianFilter(py::module &m, std::string const & suffix) {
    using Class = pcl::MedianFilter<PointT>;
    py::class_<Class, pcl::Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("applyFilter", py::overload_cast<pcl::Filter<PointT>::PointCloud &> (&Class::applyFilter), "output"_a);
    cls.def("setWindowSize", &Class::setWindowSize, "window_size"_a);
    cls.def("setMaxAllowedMovement", &Class::setMaxAllowedMovement, "max_allowed_movement"_a);
    cls.def("getWindowSize", &Class::getWindowSize);
    cls.def("getMaxAllowedMovement", &Class::getMaxAllowedMovement);
        
}

void defineFiltersMedianFilterFunctions(py::module &m) {
}

void defineFiltersMedianFilterClasses(py::module &sub_module) {
    py::module sub_module_MedianFilter = sub_module.def_submodule("MedianFilter", "Submodule MedianFilter");
    defineFiltersMedianFilter<pcl::InterestPoint>(sub_module_MedianFilter, "InterestPoint");
    defineFiltersMedianFilter<pcl::PointDEM>(sub_module_MedianFilter, "PointDEM");
    defineFiltersMedianFilter<pcl::PointNormal>(sub_module_MedianFilter, "PointNormal");
    defineFiltersMedianFilter<pcl::PointSurfel>(sub_module_MedianFilter, "PointSurfel");
    defineFiltersMedianFilter<pcl::PointWithRange>(sub_module_MedianFilter, "PointWithRange");
    defineFiltersMedianFilter<pcl::PointWithScale>(sub_module_MedianFilter, "PointWithScale");
    defineFiltersMedianFilter<pcl::PointWithViewpoint>(sub_module_MedianFilter, "PointWithViewpoint");
    defineFiltersMedianFilter<pcl::PointXYZ>(sub_module_MedianFilter, "PointXYZ");
    defineFiltersMedianFilter<pcl::PointXYZHSV>(sub_module_MedianFilter, "PointXYZHSV");
    defineFiltersMedianFilter<pcl::PointXYZI>(sub_module_MedianFilter, "PointXYZI");
    defineFiltersMedianFilter<pcl::PointXYZINormal>(sub_module_MedianFilter, "PointXYZINormal");
    defineFiltersMedianFilter<pcl::PointXYZL>(sub_module_MedianFilter, "PointXYZL");
    defineFiltersMedianFilter<pcl::PointXYZLNormal>(sub_module_MedianFilter, "PointXYZLNormal");
    defineFiltersMedianFilter<pcl::PointXYZRGB>(sub_module_MedianFilter, "PointXYZRGB");
    defineFiltersMedianFilter<pcl::PointXYZRGBA>(sub_module_MedianFilter, "PointXYZRGBA");
    defineFiltersMedianFilter<pcl::PointXYZRGBL>(sub_module_MedianFilter, "PointXYZRGBL");
    defineFiltersMedianFilter<pcl::PointXYZRGBNormal>(sub_module_MedianFilter, "PointXYZRGBNormal");
}