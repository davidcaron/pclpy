
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/project_inliers.h>



template<typename PointT>
void defineFiltersProjectInliers(py::module &m, std::string const & suffix) {
    using Class = ProjectInliers<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("model_type", &Class::getModelType, &Class::setModelType);
    cls.def_property("model_coefficients", &Class::getModelCoefficients, &Class::setModelCoefficients);
    cls.def_property("copy_all_data", &Class::getCopyAllData, &Class::setCopyAllData);
        
}



void defineFiltersProjectInliersClasses(py::module &sub_module) {
    py::module sub_module_ProjectInliers = sub_module.def_submodule("ProjectInliers", "Submodule ProjectInliers");
    defineFiltersProjectInliers<PointXYZ>(sub_module_ProjectInliers, "PointXYZ");
    defineFiltersProjectInliers<PointXYZI>(sub_module_ProjectInliers, "PointXYZI");
    defineFiltersProjectInliers<PointXYZRGB>(sub_module_ProjectInliers, "PointXYZRGB");
    defineFiltersProjectInliers<PointXYZRGBA>(sub_module_ProjectInliers, "PointXYZRGBA");
}