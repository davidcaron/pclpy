
#include <pcl/filters/local_maximum.h>



template <typename PointT>
void defineFiltersLocalMaximum(py::module &m, std::string const & suffix) {
    using Class = pcl::LocalMaximum<PointT>;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def("setRadius", &Class::setRadius, "radius"_a);
    cls.def("getRadius", &Class::getRadius);
        
}

void defineFiltersLocalMaximumFunctions(py::module &m) {
}

void defineFiltersLocalMaximumClasses(py::module &sub_module) {
    py::module sub_module_LocalMaximum = sub_module.def_submodule("LocalMaximum", "Submodule LocalMaximum");
    defineFiltersLocalMaximum<pcl::PointXYZ>(sub_module_LocalMaximum, "PointXYZ");
    defineFiltersLocalMaximum<pcl::PointXYZI>(sub_module_LocalMaximum, "PointXYZI");
    defineFiltersLocalMaximum<pcl::PointXYZRGB>(sub_module_LocalMaximum, "PointXYZRGB");
    defineFiltersLocalMaximum<pcl::PointXYZRGBA>(sub_module_LocalMaximum, "PointXYZRGBA");
}