
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/filters/project_inliers.h>



template<typename PointT>
void defineFiltersProjectInliers(py::module &m, std::string const & suffix) {
    using Class = pcl::ProjectInliers<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setModelType", &Class::setModelType, "model"_a);
    cls.def("setModelCoefficients", &Class::setModelCoefficients, "model"_a);
    cls.def("setCopyAllData", &Class::setCopyAllData, "val"_a);
    cls.def("getModelType", &Class::getModelType);
    cls.def("getModelCoefficients", &Class::getModelCoefficients);
    cls.def("getCopyAllData", &Class::getCopyAllData);
        
}

void defineFiltersProjectInliersFunctions(py::module &m) {
}

void defineFiltersProjectInliersClasses(py::module &sub_module) {
    py::module sub_module_ProjectInliers = sub_module.def_submodule("ProjectInliers", "Submodule ProjectInliers");
    defineFiltersProjectInliers<pcl::PointXYZ>(sub_module_ProjectInliers, "PointXYZ");
    defineFiltersProjectInliers<pcl::PointXYZI>(sub_module_ProjectInliers, "PointXYZI");
    defineFiltersProjectInliers<pcl::PointXYZRGB>(sub_module_ProjectInliers, "PointXYZRGB");
    defineFiltersProjectInliers<pcl::PointXYZRGBA>(sub_module_ProjectInliers, "PointXYZRGBA");
}