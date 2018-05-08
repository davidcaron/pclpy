
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

#include <pcl/filters/radius_outlier_removal.h>



template<typename PointT>
void defineFiltersRadiusOutlierRemoval(py::module &m, std::string const & suffix) {
    using Class = pcl::RadiusOutlierRemoval<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def("setRadiusSearch", &Class::setRadiusSearch, "radius"_a);
    cls.def("setMinNeighborsInRadius", &Class::setMinNeighborsInRadius, "min_pts"_a);
    cls.def("getRadiusSearch", &Class::getRadiusSearch);
    cls.def("getMinNeighborsInRadius", &Class::getMinNeighborsInRadius);
        
}

void defineFiltersRadiusOutlierRemovalFunctions(py::module &m) {
}

void defineFiltersRadiusOutlierRemovalClasses(py::module &sub_module) {
    py::module sub_module_RadiusOutlierRemoval = sub_module.def_submodule("RadiusOutlierRemoval", "Submodule RadiusOutlierRemoval");
    defineFiltersRadiusOutlierRemoval<pcl::InterestPoint>(sub_module_RadiusOutlierRemoval, "InterestPoint");
    defineFiltersRadiusOutlierRemoval<pcl::PointDEM>(sub_module_RadiusOutlierRemoval, "PointDEM");
    defineFiltersRadiusOutlierRemoval<pcl::PointNormal>(sub_module_RadiusOutlierRemoval, "PointNormal");
    defineFiltersRadiusOutlierRemoval<pcl::PointSurfel>(sub_module_RadiusOutlierRemoval, "PointSurfel");
    defineFiltersRadiusOutlierRemoval<pcl::PointWithRange>(sub_module_RadiusOutlierRemoval, "PointWithRange");
    defineFiltersRadiusOutlierRemoval<pcl::PointWithScale>(sub_module_RadiusOutlierRemoval, "PointWithScale");
    defineFiltersRadiusOutlierRemoval<pcl::PointWithViewpoint>(sub_module_RadiusOutlierRemoval, "PointWithViewpoint");
    defineFiltersRadiusOutlierRemoval<pcl::PointXYZ>(sub_module_RadiusOutlierRemoval, "PointXYZ");
    defineFiltersRadiusOutlierRemoval<pcl::PointXYZHSV>(sub_module_RadiusOutlierRemoval, "PointXYZHSV");
    defineFiltersRadiusOutlierRemoval<pcl::PointXYZI>(sub_module_RadiusOutlierRemoval, "PointXYZI");
    defineFiltersRadiusOutlierRemoval<pcl::PointXYZINormal>(sub_module_RadiusOutlierRemoval, "PointXYZINormal");
    defineFiltersRadiusOutlierRemoval<pcl::PointXYZL>(sub_module_RadiusOutlierRemoval, "PointXYZL");
    defineFiltersRadiusOutlierRemoval<pcl::PointXYZLNormal>(sub_module_RadiusOutlierRemoval, "PointXYZLNormal");
    defineFiltersRadiusOutlierRemoval<pcl::PointXYZRGB>(sub_module_RadiusOutlierRemoval, "PointXYZRGB");
    defineFiltersRadiusOutlierRemoval<pcl::PointXYZRGBA>(sub_module_RadiusOutlierRemoval, "PointXYZRGBA");
    defineFiltersRadiusOutlierRemoval<pcl::PointXYZRGBL>(sub_module_RadiusOutlierRemoval, "PointXYZRGBL");
    defineFiltersRadiusOutlierRemoval<pcl::PointXYZRGBNormal>(sub_module_RadiusOutlierRemoval, "PointXYZRGBNormal");
}