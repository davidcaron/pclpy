
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/radius_outlier_removal.h>



template<typename PointT>
void defineFiltersRadiusOutlierRemoval(py::module &m, std::string const & suffix) {
    using Class = RadiusOutlierRemoval<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def_property("radius_search", &Class::getRadiusSearch, &Class::setRadiusSearch);
    cls.def_property("min_neighbors_in_radius", &Class::getMinNeighborsInRadius, &Class::setMinNeighborsInRadius);
        
}



void defineFiltersRadiusOutlierRemovalClasses(py::module &sub_module) {
    py::module sub_module_RadiusOutlierRemoval = sub_module.def_submodule("RadiusOutlierRemoval", "Submodule RadiusOutlierRemoval");
    defineFiltersRadiusOutlierRemoval<InterestPoint>(sub_module_RadiusOutlierRemoval, "InterestPoint");
    defineFiltersRadiusOutlierRemoval<PointDEM>(sub_module_RadiusOutlierRemoval, "PointDEM");
    defineFiltersRadiusOutlierRemoval<PointNormal>(sub_module_RadiusOutlierRemoval, "PointNormal");
    defineFiltersRadiusOutlierRemoval<PointSurfel>(sub_module_RadiusOutlierRemoval, "PointSurfel");
    defineFiltersRadiusOutlierRemoval<PointWithRange>(sub_module_RadiusOutlierRemoval, "PointWithRange");
    defineFiltersRadiusOutlierRemoval<PointWithScale>(sub_module_RadiusOutlierRemoval, "PointWithScale");
    defineFiltersRadiusOutlierRemoval<PointWithViewpoint>(sub_module_RadiusOutlierRemoval, "PointWithViewpoint");
    defineFiltersRadiusOutlierRemoval<PointXYZ>(sub_module_RadiusOutlierRemoval, "PointXYZ");
    defineFiltersRadiusOutlierRemoval<PointXYZHSV>(sub_module_RadiusOutlierRemoval, "PointXYZHSV");
    defineFiltersRadiusOutlierRemoval<PointXYZI>(sub_module_RadiusOutlierRemoval, "PointXYZI");
    defineFiltersRadiusOutlierRemoval<PointXYZINormal>(sub_module_RadiusOutlierRemoval, "PointXYZINormal");
    defineFiltersRadiusOutlierRemoval<PointXYZL>(sub_module_RadiusOutlierRemoval, "PointXYZL");
    defineFiltersRadiusOutlierRemoval<PointXYZLNormal>(sub_module_RadiusOutlierRemoval, "PointXYZLNormal");
    defineFiltersRadiusOutlierRemoval<PointXYZRGB>(sub_module_RadiusOutlierRemoval, "PointXYZRGB");
    defineFiltersRadiusOutlierRemoval<PointXYZRGBA>(sub_module_RadiusOutlierRemoval, "PointXYZRGBA");
    defineFiltersRadiusOutlierRemoval<PointXYZRGBL>(sub_module_RadiusOutlierRemoval, "PointXYZRGBL");
    defineFiltersRadiusOutlierRemoval<PointXYZRGBNormal>(sub_module_RadiusOutlierRemoval, "PointXYZRGBNormal");
}