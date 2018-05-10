
#include <pcl/features/boundary.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesBoundaryEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::BoundaryEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("isBoundaryPoint", py::overload_cast<const pcl::PointCloud<PointInT> &, int, const std::vector<int> &, const Eigen::Vector4f &, const Eigen::Vector4f &, const float> (&Class::isBoundaryPoint), "cloud"_a, "q_idx"_a, "indices"_a, "u"_a, "v"_a, "angle_threshold"_a);
    cls.def("isBoundaryPoint", py::overload_cast<const pcl::PointCloud<PointInT> &, const PointInT &, const std::vector<int> &, const Eigen::Vector4f &, const Eigen::Vector4f &, const float> (&Class::isBoundaryPoint), "cloud"_a, "q_point"_a, "indices"_a, "u"_a, "v"_a, "angle_threshold"_a);
    cls.def("setAngleThreshold", &Class::setAngleThreshold, "angle"_a);
    cls.def("getAngleThreshold", &Class::getAngleThreshold);
    cls.def("getCoordinateSystemOnPlane", &Class::getCoordinateSystemOnPlane, "p_coeff"_a, "u"_a, "v"_a);
        
}

void defineFeaturesBoundaryFunctions(py::module &m) {
}

void defineFeaturesBoundaryClasses(py::module &sub_module) {
    py::module sub_module_BoundaryEstimation = sub_module.def_submodule("BoundaryEstimation", "Submodule BoundaryEstimation");
    defineFeaturesBoundaryEstimation<pcl::PointNormal, pcl::Normal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointNormal_Normal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointNormal, pcl::PointNormal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointNormal_PointNormal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointNormal, pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointNormal_PointXYZRGBNormal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZ_Normal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZ_PointNormal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZ, pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZ_PointXYZRGBNormal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZI, pcl::Normal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZI_Normal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZI_PointNormal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZI, pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZI_PointXYZRGBNormal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBA_Normal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBA_PointNormal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBA_PointXYZRGBNormal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZRGBNormal, pcl::Normal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBNormal_Normal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZRGBNormal, pcl::PointNormal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBNormal_PointNormal_Boundary");
    defineFeaturesBoundaryEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBNormal_PointXYZRGBNormal_Boundary");
}