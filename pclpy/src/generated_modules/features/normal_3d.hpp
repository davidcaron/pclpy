
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/features/normal_3d.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesNormalEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::NormalEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computePointNormal", py::overload_cast<const pcl::PointCloud<PointInT> &, const std::vector<int> &, Eigen::Vector4f &, float &> (&Class::computePointNormal), "cloud"_a, "indices"_a, "plane_parameters"_a, "curvature"_a);
    cls.def("computePointNormal", py::overload_cast<const pcl::PointCloud<PointInT> &, const std::vector<int> &, float &, float &, float &, float &> (&Class::computePointNormal), "cloud"_a, "indices"_a, "nx"_a, "ny"_a, "nz"_a, "curvature"_a);
    cls.def("useSensorOriginAsViewPoint", &Class::useSensorOriginAsViewPoint);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setViewPoint", &Class::setViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
    cls.def("getViewPoint", &Class::getViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
        
}

void defineFeaturesNormal3dFunctions(py::module &m) {
}

void defineFeaturesNormal3dClasses(py::module &sub_module) {
    py::module sub_module_NormalEstimation = sub_module.def_submodule("NormalEstimation", "Submodule NormalEstimation");
    defineFeaturesNormalEstimation<pcl::PointNormal, pcl::Normal>(sub_module_NormalEstimation, "PointNormal_Normal");
    defineFeaturesNormalEstimation<pcl::PointNormal, pcl::PointNormal>(sub_module_NormalEstimation, "PointNormal_PointNormal");
    defineFeaturesNormalEstimation<pcl::PointNormal, pcl::PointXYZRGBNormal>(sub_module_NormalEstimation, "PointNormal_PointXYZRGBNormal");
    defineFeaturesNormalEstimation<pcl::PointSurfel, pcl::Normal>(sub_module_NormalEstimation, "PointSurfel_Normal");
    defineFeaturesNormalEstimation<pcl::PointSurfel, pcl::PointNormal>(sub_module_NormalEstimation, "PointSurfel_PointNormal");
    defineFeaturesNormalEstimation<pcl::PointSurfel, pcl::PointXYZRGBNormal>(sub_module_NormalEstimation, "PointSurfel_PointXYZRGBNormal");
    defineFeaturesNormalEstimation<pcl::PointXYZ, pcl::Normal>(sub_module_NormalEstimation, "PointXYZ_Normal");
    defineFeaturesNormalEstimation<pcl::PointXYZ, pcl::PointNormal>(sub_module_NormalEstimation, "PointXYZ_PointNormal");
    defineFeaturesNormalEstimation<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_NormalEstimation, "PointXYZ_PointXYZRGBNormal");
    defineFeaturesNormalEstimation<pcl::PointXYZI, pcl::Normal>(sub_module_NormalEstimation, "PointXYZI_Normal");
    defineFeaturesNormalEstimation<pcl::PointXYZI, pcl::PointNormal>(sub_module_NormalEstimation, "PointXYZI_PointNormal");
    defineFeaturesNormalEstimation<pcl::PointXYZI, pcl::PointXYZRGBNormal>(sub_module_NormalEstimation, "PointXYZI_PointXYZRGBNormal");
    defineFeaturesNormalEstimation<pcl::PointXYZRGB, pcl::Normal>(sub_module_NormalEstimation, "PointXYZRGB_Normal");
    defineFeaturesNormalEstimation<pcl::PointXYZRGB, pcl::PointNormal>(sub_module_NormalEstimation, "PointXYZRGB_PointNormal");
    defineFeaturesNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_NormalEstimation, "PointXYZRGB_PointXYZRGBNormal");
    defineFeaturesNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>(sub_module_NormalEstimation, "PointXYZRGBA_Normal");
    defineFeaturesNormalEstimation<pcl::PointXYZRGBA, pcl::PointNormal>(sub_module_NormalEstimation, "PointXYZRGBA_PointNormal");
    defineFeaturesNormalEstimation<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_NormalEstimation, "PointXYZRGBA_PointXYZRGBNormal");
    defineFeaturesNormal3dFunctions(sub_module);
}