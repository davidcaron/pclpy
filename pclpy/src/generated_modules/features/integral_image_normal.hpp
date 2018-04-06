
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/integral_image_normal.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesIntegralImageNormalEstimation(py::module &m, std::string const & suffix) {
    using Class = IntegralImageNormalEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::BorderPolicy>(cls, "border_policy")
        .value("BORDER_POLICY_IGNORE", Class::BorderPolicy::BORDER_POLICY_IGNORE)
        .value("BORDER_POLICY_MIRROR", Class::BorderPolicy::BORDER_POLICY_MIRROR)
        .export_values();
    py::enum_<Class::NormalEstimationMethod>(cls, "normal_estimation_method")
        .value("COVARIANCE_MATRIX", Class::NormalEstimationMethod::COVARIANCE_MATRIX)
        .value("AVERAGE_3D_GRADIENT", Class::NormalEstimationMethod::AVERAGE_3D_GRADIENT)
        .value("AVERAGE_DEPTH_CHANGE", Class::NormalEstimationMethod::AVERAGE_DEPTH_CHANGE)
        .value("SIMPLE_3D_GRADIENT", Class::NormalEstimationMethod::SIMPLE_3D_GRADIENT)
        .export_values();
    cls.def(py::init<>());
    cls.def("set_rect_size", &Class::setRectSize);
    cls.def("set_border_policy", &Class::setBorderPolicy);
    cls.def("set_max_depth_change_factor", &Class::setMaxDepthChangeFactor);
    cls.def("set_normal_smoothing_size", &Class::setNormalSmoothingSize);
    cls.def("set_normal_estimation_method", &Class::setNormalEstimationMethod);
    cls.def("set_depth_dependent_smoothing", &Class::setDepthDependentSmoothing);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def_property("view_point", &Class::getViewPoint, &Class::setViewPoint);
    cls.def("compute_point_normal", py::overload_cast<const int, const int, const unsigned, PointOutT &> (&Class::computePointNormal));
    cls.def("compute_point_normal_mirror", &Class::computePointNormalMirror);
    cls.def("use_sensor_origin_as_view_point", &Class::useSensorOriginAsViewPoint);
        
}

void defineFeaturesIntegralImageNormalClasses(py::module &sub_module) {
    py::module sub_module_IntegralImageNormalEstimation = sub_module.def_submodule("IntegralImageNormalEstimation", "Submodule IntegralImageNormalEstimation");
    defineFeaturesIntegralImageNormalEstimation<PointXYZ, Normal>(sub_module_IntegralImageNormalEstimation, "PointXYZ_Normal");
    defineFeaturesIntegralImageNormalEstimation<PointXYZ, PointXYZRGBNormal>(sub_module_IntegralImageNormalEstimation, "PointXYZ_PointXYZRGBNormal");
    defineFeaturesIntegralImageNormalEstimation<PointXYZRGB, Normal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGB_Normal");
    defineFeaturesIntegralImageNormalEstimation<PointXYZRGB, PointXYZRGBNormal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGB_PointXYZRGBNormal");
    defineFeaturesIntegralImageNormalEstimation<PointXYZRGBA, Normal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGBA_Normal");
    defineFeaturesIntegralImageNormalEstimation<PointXYZRGBA, PointXYZRGBNormal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGBA_PointXYZRGBNormal");
    defineFeaturesIntegralImageNormalEstimation<PointXYZRGBNormal, Normal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGBNormal_Normal");
    defineFeaturesIntegralImageNormalEstimation<PointXYZRGBNormal, PointXYZRGBNormal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGBNormal_PointXYZRGBNormal");
}