
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/frustum_culling.h>



template <typename PointT>
void defineFiltersFrustumCulling(py::module &m, std::string const & suffix) {
    using Class = FrustumCulling<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def_property("camera_pose", &Class::getCameraPose, &Class::setCameraPose);
    cls.def_property("horizontal_fov", &Class::getHorizontalFOV, &Class::setHorizontalFOV);
    cls.def_property("vertical_fov", &Class::getVerticalFOV, &Class::setVerticalFOV);
    cls.def_property("near_plane_distance", &Class::getNearPlaneDistance, &Class::setNearPlaneDistance);
    cls.def_property("far_plane_distance", &Class::getFarPlaneDistance, &Class::setFarPlaneDistance);
        
}

void defineFiltersFrustumCullingClasses(py::module &sub_module) {
    py::module sub_module_FrustumCulling = sub_module.def_submodule("FrustumCulling", "Submodule FrustumCulling");
    defineFiltersFrustumCulling<InterestPoint>(sub_module_FrustumCulling, "InterestPoint");
    defineFiltersFrustumCulling<PointDEM>(sub_module_FrustumCulling, "PointDEM");
    defineFiltersFrustumCulling<PointNormal>(sub_module_FrustumCulling, "PointNormal");
    defineFiltersFrustumCulling<PointSurfel>(sub_module_FrustumCulling, "PointSurfel");
    defineFiltersFrustumCulling<PointWithRange>(sub_module_FrustumCulling, "PointWithRange");
    defineFiltersFrustumCulling<PointWithScale>(sub_module_FrustumCulling, "PointWithScale");
    defineFiltersFrustumCulling<PointWithViewpoint>(sub_module_FrustumCulling, "PointWithViewpoint");
    defineFiltersFrustumCulling<PointXYZ>(sub_module_FrustumCulling, "PointXYZ");
    defineFiltersFrustumCulling<PointXYZHSV>(sub_module_FrustumCulling, "PointXYZHSV");
    defineFiltersFrustumCulling<PointXYZI>(sub_module_FrustumCulling, "PointXYZI");
    defineFiltersFrustumCulling<PointXYZINormal>(sub_module_FrustumCulling, "PointXYZINormal");
    defineFiltersFrustumCulling<PointXYZL>(sub_module_FrustumCulling, "PointXYZL");
    defineFiltersFrustumCulling<PointXYZLNormal>(sub_module_FrustumCulling, "PointXYZLNormal");
    defineFiltersFrustumCulling<PointXYZRGB>(sub_module_FrustumCulling, "PointXYZRGB");
    defineFiltersFrustumCulling<PointXYZRGBA>(sub_module_FrustumCulling, "PointXYZRGBA");
    defineFiltersFrustumCulling<PointXYZRGBL>(sub_module_FrustumCulling, "PointXYZRGBL");
    defineFiltersFrustumCulling<PointXYZRGBNormal>(sub_module_FrustumCulling, "PointXYZRGBNormal");
}