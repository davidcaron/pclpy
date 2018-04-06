
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/normal_3d.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesNormalEstimation(py::module &m, std::string const & suffix) {
    using Class = NormalEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def_property("view_point", &Class::getViewPoint, &Class::setViewPoint);
    cls.def("compute_point_normal", py::overload_cast<const pcl::PointCloud<PointInT> &, const std::vector<int> &, Eigen::Vector4f &, float &> (&Class::computePointNormal));
    cls.def("compute_point_normal", py::overload_cast<const pcl::PointCloud<PointInT> &, const std::vector<int> &, float &, float &, float &, float &> (&Class::computePointNormal));
    cls.def("use_sensor_origin_as_view_point", &Class::useSensorOriginAsViewPoint);
        
}

void defineFeaturesNormal3dClasses(py::module &sub_module) {
    py::module sub_module_NormalEstimation = sub_module.def_submodule("NormalEstimation", "Submodule NormalEstimation");
    defineFeaturesNormalEstimation<PointNormal, Normal>(sub_module_NormalEstimation, "PointNormal_Normal");
    defineFeaturesNormalEstimation<PointNormal, PointNormal>(sub_module_NormalEstimation, "PointNormal_PointNormal");
    defineFeaturesNormalEstimation<PointNormal, PointXYZRGBNormal>(sub_module_NormalEstimation, "PointNormal_PointXYZRGBNormal");
    defineFeaturesNormalEstimation<PointSurfel, Normal>(sub_module_NormalEstimation, "PointSurfel_Normal");
    defineFeaturesNormalEstimation<PointSurfel, PointNormal>(sub_module_NormalEstimation, "PointSurfel_PointNormal");
    defineFeaturesNormalEstimation<PointSurfel, PointXYZRGBNormal>(sub_module_NormalEstimation, "PointSurfel_PointXYZRGBNormal");
    defineFeaturesNormalEstimation<PointXYZ, Normal>(sub_module_NormalEstimation, "PointXYZ_Normal");
    defineFeaturesNormalEstimation<PointXYZ, PointNormal>(sub_module_NormalEstimation, "PointXYZ_PointNormal");
    defineFeaturesNormalEstimation<PointXYZ, PointXYZRGBNormal>(sub_module_NormalEstimation, "PointXYZ_PointXYZRGBNormal");
    defineFeaturesNormalEstimation<PointXYZI, Normal>(sub_module_NormalEstimation, "PointXYZI_Normal");
    defineFeaturesNormalEstimation<PointXYZI, PointNormal>(sub_module_NormalEstimation, "PointXYZI_PointNormal");
    defineFeaturesNormalEstimation<PointXYZI, PointXYZRGBNormal>(sub_module_NormalEstimation, "PointXYZI_PointXYZRGBNormal");
    defineFeaturesNormalEstimation<PointXYZRGB, Normal>(sub_module_NormalEstimation, "PointXYZRGB_Normal");
    defineFeaturesNormalEstimation<PointXYZRGB, PointNormal>(sub_module_NormalEstimation, "PointXYZRGB_PointNormal");
    defineFeaturesNormalEstimation<PointXYZRGB, PointXYZRGBNormal>(sub_module_NormalEstimation, "PointXYZRGB_PointXYZRGBNormal");
    defineFeaturesNormalEstimation<PointXYZRGBA, Normal>(sub_module_NormalEstimation, "PointXYZRGBA_Normal");
    defineFeaturesNormalEstimation<PointXYZRGBA, PointNormal>(sub_module_NormalEstimation, "PointXYZRGBA_PointNormal");
    defineFeaturesNormalEstimation<PointXYZRGBA, PointXYZRGBNormal>(sub_module_NormalEstimation, "PointXYZRGBA_PointXYZRGBNormal");
}