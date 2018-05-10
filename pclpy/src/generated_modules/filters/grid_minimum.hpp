
#include <pcl/filters/grid_minimum.h>



template <typename PointT>
void defineFiltersGridMinimum(py::module &m, std::string const & suffix) {
    using Class = pcl::GridMinimum<PointT>;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<float>(), "resolution"_a);
    cls.def("setResolution", &Class::setResolution, "resolution"_a);
    cls.def("getResolution", &Class::getResolution);
        
}

void defineFiltersGridMinimumFunctions(py::module &m) {
}

void defineFiltersGridMinimumClasses(py::module &sub_module) {
    py::module sub_module_GridMinimum = sub_module.def_submodule("GridMinimum", "Submodule GridMinimum");
    defineFiltersGridMinimum<pcl::InterestPoint>(sub_module_GridMinimum, "InterestPoint");
    defineFiltersGridMinimum<pcl::PointDEM>(sub_module_GridMinimum, "PointDEM");
    defineFiltersGridMinimum<pcl::PointNormal>(sub_module_GridMinimum, "PointNormal");
    defineFiltersGridMinimum<pcl::PointSurfel>(sub_module_GridMinimum, "PointSurfel");
    defineFiltersGridMinimum<pcl::PointWithRange>(sub_module_GridMinimum, "PointWithRange");
    defineFiltersGridMinimum<pcl::PointWithScale>(sub_module_GridMinimum, "PointWithScale");
    defineFiltersGridMinimum<pcl::PointWithViewpoint>(sub_module_GridMinimum, "PointWithViewpoint");
    defineFiltersGridMinimum<pcl::PointXYZ>(sub_module_GridMinimum, "PointXYZ");
    defineFiltersGridMinimum<pcl::PointXYZHSV>(sub_module_GridMinimum, "PointXYZHSV");
    defineFiltersGridMinimum<pcl::PointXYZI>(sub_module_GridMinimum, "PointXYZI");
    defineFiltersGridMinimum<pcl::PointXYZINormal>(sub_module_GridMinimum, "PointXYZINormal");
    defineFiltersGridMinimum<pcl::PointXYZL>(sub_module_GridMinimum, "PointXYZL");
    defineFiltersGridMinimum<pcl::PointXYZLNormal>(sub_module_GridMinimum, "PointXYZLNormal");
    defineFiltersGridMinimum<pcl::PointXYZRGB>(sub_module_GridMinimum, "PointXYZRGB");
    defineFiltersGridMinimum<pcl::PointXYZRGBA>(sub_module_GridMinimum, "PointXYZRGBA");
    defineFiltersGridMinimum<pcl::PointXYZRGBL>(sub_module_GridMinimum, "PointXYZRGBL");
    defineFiltersGridMinimum<pcl::PointXYZRGBNormal>(sub_module_GridMinimum, "PointXYZRGBNormal");
}