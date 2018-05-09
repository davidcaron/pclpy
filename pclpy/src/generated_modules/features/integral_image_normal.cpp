
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/integral_image_normal.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesIntegralImageNormalEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::IntegralImageNormalEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::BorderPolicy>(cls, "BorderPolicy")
        .value("BORDER_POLICY_IGNORE", Class::BorderPolicy::BORDER_POLICY_IGNORE)
        .value("BORDER_POLICY_MIRROR", Class::BorderPolicy::BORDER_POLICY_MIRROR)
        .export_values();
    py::enum_<Class::NormalEstimationMethod>(cls, "NormalEstimationMethod")
        .value("COVARIANCE_MATRIX", Class::NormalEstimationMethod::COVARIANCE_MATRIX)
        .value("AVERAGE_3D_GRADIENT", Class::NormalEstimationMethod::AVERAGE_3D_GRADIENT)
        .value("AVERAGE_DEPTH_CHANGE", Class::NormalEstimationMethod::AVERAGE_DEPTH_CHANGE)
        .value("SIMPLE_3D_GRADIENT", Class::NormalEstimationMethod::SIMPLE_3D_GRADIENT)
        .export_values();
    cls.def(py::init<>());
    cls.def("computePointNormal", py::overload_cast<const int, const int, const unsigned, PointOutT &> (&Class::computePointNormal), "pos_x"_a, "pos_y"_a, "point_index"_a, "normal"_a);
    cls.def("computePointNormalMirror", &Class::computePointNormalMirror, "pos_x"_a, "pos_y"_a, "point_index"_a, "normal"_a);
    cls.def("useSensorOriginAsViewPoint", &Class::useSensorOriginAsViewPoint);
    cls.def("setRectSize", &Class::setRectSize, "width"_a, "height"_a);
    cls.def("setBorderPolicy", &Class::setBorderPolicy, "border_policy"_a);
    cls.def("setMaxDepthChangeFactor", &Class::setMaxDepthChangeFactor, "max_depth_change_factor"_a);
    cls.def("setNormalSmoothingSize", &Class::setNormalSmoothingSize, "normal_smoothing_size"_a);
    cls.def("setNormalEstimationMethod", &Class::setNormalEstimationMethod, "normal_estimation_method"_a);
    cls.def("setDepthDependentSmoothing", &Class::setDepthDependentSmoothing, "use_depth_dependent_smoothing"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setViewPoint", &Class::setViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
    cls.def("getDistanceMap", &Class::getDistanceMap);
    cls.def("getViewPoint", &Class::getViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
        
}

void defineFeaturesIntegralImageNormalFunctions(py::module &m) {
}

void defineFeaturesIntegralImageNormalClasses(py::module &sub_module) {
    py::module sub_module_IntegralImageNormalEstimation = sub_module.def_submodule("IntegralImageNormalEstimation", "Submodule IntegralImageNormalEstimation");
    defineFeaturesIntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>(sub_module_IntegralImageNormalEstimation, "PointXYZ_Normal");
    defineFeaturesIntegralImageNormalEstimation<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_IntegralImageNormalEstimation, "PointXYZ_PointXYZRGBNormal");
    defineFeaturesIntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGB_Normal");
    defineFeaturesIntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGB_PointXYZRGBNormal");
    defineFeaturesIntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGBA_Normal");
    defineFeaturesIntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGBA_PointXYZRGBNormal");
    defineFeaturesIntegralImageNormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGBNormal_Normal");
    defineFeaturesIntegralImageNormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(sub_module_IntegralImageNormalEstimation, "PointXYZRGBNormal_PointXYZRGBNormal");
}