
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/filters/frustum_culling.h>



template <typename PointT>
void defineFiltersFrustumCulling(py::module &m, std::string const & suffix) {
    using Class = pcl::FrustumCulling<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def("setCameraPose", &Class::setCameraPose, "camera_pose"_a);
    cls.def("setHorizontalFOV", &Class::setHorizontalFOV, "hfov"_a);
    cls.def("setVerticalFOV", &Class::setVerticalFOV, "vfov"_a);
    cls.def("setNearPlaneDistance", &Class::setNearPlaneDistance, "np_dist"_a);
    cls.def("setFarPlaneDistance", &Class::setFarPlaneDistance, "fp_dist"_a);
    cls.def("getCameraPose", &Class::getCameraPose);
    cls.def("getHorizontalFOV", &Class::getHorizontalFOV);
    cls.def("getVerticalFOV", &Class::getVerticalFOV);
    cls.def("getNearPlaneDistance", &Class::getNearPlaneDistance);
    cls.def("getFarPlaneDistance", &Class::getFarPlaneDistance);
        
}

void defineFiltersFrustumCullingFunctions(py::module &m) {
}

void defineFiltersFrustumCullingClasses(py::module &sub_module) {
    py::module sub_module_FrustumCulling = sub_module.def_submodule("FrustumCulling", "Submodule FrustumCulling");
    defineFiltersFrustumCulling<pcl::InterestPoint>(sub_module_FrustumCulling, "InterestPoint");
    defineFiltersFrustumCulling<pcl::PointDEM>(sub_module_FrustumCulling, "PointDEM");
    defineFiltersFrustumCulling<pcl::PointNormal>(sub_module_FrustumCulling, "PointNormal");
    defineFiltersFrustumCulling<pcl::PointSurfel>(sub_module_FrustumCulling, "PointSurfel");
    defineFiltersFrustumCulling<pcl::PointWithRange>(sub_module_FrustumCulling, "PointWithRange");
    defineFiltersFrustumCulling<pcl::PointWithScale>(sub_module_FrustumCulling, "PointWithScale");
    defineFiltersFrustumCulling<pcl::PointWithViewpoint>(sub_module_FrustumCulling, "PointWithViewpoint");
    defineFiltersFrustumCulling<pcl::PointXYZ>(sub_module_FrustumCulling, "PointXYZ");
    defineFiltersFrustumCulling<pcl::PointXYZHSV>(sub_module_FrustumCulling, "PointXYZHSV");
    defineFiltersFrustumCulling<pcl::PointXYZI>(sub_module_FrustumCulling, "PointXYZI");
    defineFiltersFrustumCulling<pcl::PointXYZINormal>(sub_module_FrustumCulling, "PointXYZINormal");
    defineFiltersFrustumCulling<pcl::PointXYZL>(sub_module_FrustumCulling, "PointXYZL");
    defineFiltersFrustumCulling<pcl::PointXYZLNormal>(sub_module_FrustumCulling, "PointXYZLNormal");
    defineFiltersFrustumCulling<pcl::PointXYZRGB>(sub_module_FrustumCulling, "PointXYZRGB");
    defineFiltersFrustumCulling<pcl::PointXYZRGBA>(sub_module_FrustumCulling, "PointXYZRGBA");
    defineFiltersFrustumCulling<pcl::PointXYZRGBL>(sub_module_FrustumCulling, "PointXYZRGBL");
    defineFiltersFrustumCulling<pcl::PointXYZRGBNormal>(sub_module_FrustumCulling, "PointXYZRGBNormal");
}