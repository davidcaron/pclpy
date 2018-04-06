
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/local_maximum.h>



template <typename PointT>
void defineFiltersLocalMaximum(py::module &m, std::string const & suffix) {
    using Class = LocalMaximum<PointT>;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def_property("radius", &Class::getRadius, &Class::setRadius);
        
}

void defineFiltersLocalMaximumClasses(py::module &sub_module) {
    py::module sub_module_LocalMaximum = sub_module.def_submodule("LocalMaximum", "Submodule LocalMaximum");
    defineFiltersLocalMaximum<PointXYZ>(sub_module_LocalMaximum, "PointXYZ");
    defineFiltersLocalMaximum<PointXYZI>(sub_module_LocalMaximum, "PointXYZI");
    defineFiltersLocalMaximum<PointXYZRGB>(sub_module_LocalMaximum, "PointXYZRGB");
    defineFiltersLocalMaximum<PointXYZRGBA>(sub_module_LocalMaximum, "PointXYZRGBA");
}