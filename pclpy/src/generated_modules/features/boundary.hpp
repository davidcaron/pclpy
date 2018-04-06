
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/boundary.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesBoundaryEstimation(py::module &m, std::string const & suffix) {
    using Class = BoundaryEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("angle_threshold", &Class::getAngleThreshold, &Class::setAngleThreshold);
    cls.def("is_boundary_point", py::overload_cast<const pcl::PointCloud<PointInT> &, int, const std::vector<int> &, const Eigen::Vector4f &, const Eigen::Vector4f &, const float> (&Class::isBoundaryPoint));
    cls.def("is_boundary_point", py::overload_cast<const pcl::PointCloud<PointInT> &, const PointInT &, const std::vector<int> &, const Eigen::Vector4f &, const Eigen::Vector4f &, const float> (&Class::isBoundaryPoint));
        
}

void defineFeaturesBoundaryClasses(py::module &sub_module) {
    py::module sub_module_BoundaryEstimation = sub_module.def_submodule("BoundaryEstimation", "Submodule BoundaryEstimation");
    defineFeaturesBoundaryEstimation<PointNormal, Normal, Boundary>(sub_module_BoundaryEstimation, "PointNormal_Normal_Boundary");
    defineFeaturesBoundaryEstimation<PointNormal, PointNormal, Boundary>(sub_module_BoundaryEstimation, "PointNormal_PointNormal_Boundary");
    defineFeaturesBoundaryEstimation<PointNormal, PointXYZRGBNormal, Boundary>(sub_module_BoundaryEstimation, "PointNormal_PointXYZRGBNormal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZ, Normal, Boundary>(sub_module_BoundaryEstimation, "PointXYZ_Normal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZ, PointNormal, Boundary>(sub_module_BoundaryEstimation, "PointXYZ_PointNormal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZ, PointXYZRGBNormal, Boundary>(sub_module_BoundaryEstimation, "PointXYZ_PointXYZRGBNormal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZI, Normal, Boundary>(sub_module_BoundaryEstimation, "PointXYZI_Normal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZI, PointNormal, Boundary>(sub_module_BoundaryEstimation, "PointXYZI_PointNormal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZI, PointXYZRGBNormal, Boundary>(sub_module_BoundaryEstimation, "PointXYZI_PointXYZRGBNormal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZRGBA, Normal, Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBA_Normal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZRGBA, PointNormal, Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBA_PointNormal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZRGBA, PointXYZRGBNormal, Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBA_PointXYZRGBNormal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZRGBNormal, Normal, Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBNormal_Normal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZRGBNormal, PointNormal, Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBNormal_PointNormal_Boundary");
    defineFeaturesBoundaryEstimation<PointXYZRGBNormal, PointXYZRGBNormal, Boundary>(sub_module_BoundaryEstimation, "PointXYZRGBNormal_PointXYZRGBNormal_Boundary");
}