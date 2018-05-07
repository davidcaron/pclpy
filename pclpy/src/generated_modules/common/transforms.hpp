
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/common/transforms.h>



template <typename PointT>
void defineCommonTransformsFunctions1(py::module &m) {
    m.def("getPrincipalTransformation", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Affine3f &> (&pcl::getPrincipalTransformation<PointT>), "cloud"_a, "transform"_a);
    m.def("transformPoint", py::overload_cast<const PointT &, const Eigen::Affine3f &> (&pcl::transformPoint<PointT>), "point"_a, "transform"_a);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, pcl::PointCloud<PointT> &, const Eigen::Affine3f &, bool> (&pcl::transformPointCloud<PointT>), "cloud_in"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, pcl::PointCloud<PointT> &, const Eigen::Affine3f &, bool> (&pcl::transformPointCloud<PointT>), "cloud_in"_a, "indices"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, pcl::PointCloud<PointT> &, const Eigen::Affine3f &, bool> (&pcl::transformPointCloud<PointT>), "cloud_in"_a, "indices"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, pcl::PointCloud<PointT> &, const Eigen::Matrix4f &, bool> (&pcl::transformPointCloud<PointT>), "cloud_in"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, pcl::PointCloud<PointT> &, const Eigen::Matrix4f &, bool> (&pcl::transformPointCloud<PointT>), "cloud_in"_a, "indices"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, pcl::PointCloud<PointT> &, const Eigen::Matrix4f &, bool> (&pcl::transformPointCloud<PointT>), "cloud_in"_a, "indices"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, pcl::PointCloud<PointT> &, const Eigen::Vector3f &, const Eigen::Quaternionf &, bool> (&pcl::transformPointCloud<PointT>), "cloud_in"_a, "cloud_out"_a, "offset"_a, "rotation"_a, "copy_all_fields"_a=true);
}

template <typename PointT, typename Scalar>
void defineCommonTransformsFunctions2(py::module &m) {
    m.def("getPrincipalTransformation", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Transform<Scalar, 3, Eigen::Affine> &> (&pcl::getPrincipalTransformation<PointT, Scalar>), "cloud"_a, "transform"_a);
    m.def("transformPoint", py::overload_cast<const PointT &, const Eigen::Transform<Scalar, 3, Eigen::Affine> &> (&pcl::transformPoint<PointT, Scalar>), "point"_a, "transform"_a);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, pcl::PointCloud<PointT> &, const Eigen::Transform<Scalar, 3, Eigen::Affine> &, bool> (&pcl::transformPointCloud<PointT, Scalar>), "cloud_in"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, pcl::PointCloud<PointT> &, const Eigen::Transform<Scalar, 3, Eigen::Affine> &, bool> (&pcl::transformPointCloud<PointT, Scalar>), "cloud_in"_a, "indices"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, pcl::PointCloud<PointT> &, const Eigen::Transform<Scalar, 3, Eigen::Affine> &, bool> (&pcl::transformPointCloud<PointT, Scalar>), "cloud_in"_a, "indices"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, pcl::PointCloud<PointT> &, const Eigen::Matrix<Scalar, 4, 4> &, bool> (&pcl::transformPointCloud<PointT, Scalar>), "cloud_in"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, pcl::PointCloud<PointT> &, const Eigen::Matrix<Scalar, 4, 4> &, bool> (&pcl::transformPointCloud<PointT, Scalar>), "cloud_in"_a, "indices"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, pcl::PointCloud<PointT> &, const Eigen::Matrix<Scalar, 4, 4> &, bool> (&pcl::transformPointCloud<PointT, Scalar>), "cloud_in"_a, "indices"_a, "cloud_out"_a, "transform"_a, "copy_all_fields"_a=true);
    m.def("transformPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, pcl::PointCloud<PointT> &, const Eigen::Matrix<Scalar, 3, 1> &, const Eigen::Quaternion<Scalar> &, bool> (&pcl::transformPointCloud<PointT, Scalar>), "cloud_in"_a, "cloud_out"_a, "offset"_a, "rotation"_a, "copy_all_fields"_a=true);
}

void defineCommonTransformsFunctions(py::module &m) {
    defineCommonTransformsFunctions1<pcl::PointXYZ>(m);
    defineCommonTransformsFunctions1<pcl::PointXYZI>(m);
    defineCommonTransformsFunctions1<pcl::PointXYZL>(m);
    defineCommonTransformsFunctions1<pcl::PointXYZRGBA>(m);
    defineCommonTransformsFunctions1<pcl::PointXYZRGB>(m);
    defineCommonTransformsFunctions1<pcl::PointXYZRGBL>(m);
    defineCommonTransformsFunctions1<pcl::PointXYZHSV>(m);
    defineCommonTransformsFunctions1<pcl::InterestPoint>(m);
    defineCommonTransformsFunctions1<pcl::PointNormal>(m);
    defineCommonTransformsFunctions1<pcl::PointXYZRGBNormal>(m);
    defineCommonTransformsFunctions1<pcl::PointXYZINormal>(m);
    defineCommonTransformsFunctions1<pcl::PointXYZLNormal>(m);
    defineCommonTransformsFunctions1<pcl::PointWithRange>(m);
    defineCommonTransformsFunctions1<pcl::PointWithViewpoint>(m);
    defineCommonTransformsFunctions1<pcl::PointWithScale>(m);
    defineCommonTransformsFunctions1<pcl::PointSurfel>(m);
    defineCommonTransformsFunctions1<pcl::PointDEM>(m);
    defineCommonTransformsFunctions2<pcl::PointXYZ, float>(m);
    defineCommonTransformsFunctions2<pcl::PointXYZI, float>(m);
    defineCommonTransformsFunctions2<pcl::PointXYZL, float>(m);
    defineCommonTransformsFunctions2<pcl::PointXYZRGBA, float>(m);
    defineCommonTransformsFunctions2<pcl::PointXYZRGB, float>(m);
    defineCommonTransformsFunctions2<pcl::PointXYZRGBL, float>(m);
    defineCommonTransformsFunctions2<pcl::PointXYZHSV, float>(m);
    defineCommonTransformsFunctions2<pcl::InterestPoint, float>(m);
    defineCommonTransformsFunctions2<pcl::PointNormal, float>(m);
    defineCommonTransformsFunctions2<pcl::PointXYZRGBNormal, float>(m);
    defineCommonTransformsFunctions2<pcl::PointXYZINormal, float>(m);
    defineCommonTransformsFunctions2<pcl::PointXYZLNormal, float>(m);
    defineCommonTransformsFunctions2<pcl::PointWithRange, float>(m);
    defineCommonTransformsFunctions2<pcl::PointWithViewpoint, float>(m);
    defineCommonTransformsFunctions2<pcl::PointWithScale, float>(m);
    defineCommonTransformsFunctions2<pcl::PointSurfel, float>(m);
    defineCommonTransformsFunctions2<pcl::PointDEM, float>(m);
}

void defineCommonTransformsClasses(py::module &sub_module) {
    defineCommonTransformsFunctions(sub_module);
}