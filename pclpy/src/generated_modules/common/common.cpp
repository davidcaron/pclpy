
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/common/common.h>



void defineCommonCommonFunctions1(py::module &m) {
    m.def("getAngle3D", py::overload_cast<const Eigen::Vector4f &, const Eigen::Vector4f &, const bool> (&pcl::getAngle3D), "v1"_a, "v2"_a, "in_degree"_a=false);
    m.def("getAngle3D", py::overload_cast<const Eigen::Vector3f &, const Eigen::Vector3f &, const bool> (&pcl::getAngle3D), "v1"_a, "v2"_a, "in_degree"_a=false);
    m.def("getMeanStd", py::overload_cast<const std::vector<float> &, double &, double &> (&pcl::getMeanStd), "values"_a, "mean"_a, "stddev"_a);
    m.def("getMeanStdDev", py::overload_cast<const std::vector<float> &, double &, double &> (&pcl::getMeanStdDev), "values"_a, "mean"_a, "stddev"_a);
}

template<typename PointT>
void defineCommonCommonFunctions2(py::module &m) {
    m.def("calculatePolygonArea", py::overload_cast<const pcl::PointCloud<PointT> &> (&pcl::calculatePolygonArea<PointT>), "polygon"_a);
    m.def("getCircumcircleRadius", py::overload_cast<const PointT &, const PointT &, const PointT &> (&pcl::getCircumcircleRadius<PointT>), "pa"_a, "pb"_a, "pc"_a);
    m.def("getMaxDistance", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Vector4f &, Eigen::Vector4f &> (&pcl::getMaxDistance<PointT>), "cloud"_a, "pivot_pt"_a, "max_pt"_a);
    m.def("getMaxDistance", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Vector4f &, Eigen::Vector4f &> (&pcl::getMaxDistance<PointT>), "cloud"_a, "indices"_a, "pivot_pt"_a, "max_pt"_a);
    m.def("getMinMax3D", py::overload_cast<const pcl::PointCloud<PointT> &, PointT &, PointT &> (&pcl::getMinMax3D<PointT>), "cloud"_a, "min_pt"_a, "max_pt"_a);
    m.def("getMinMax3D", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Vector4f &, Eigen::Vector4f &> (&pcl::getMinMax3D<PointT>), "cloud"_a, "min_pt"_a, "max_pt"_a);
    m.def("getMinMax3D", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Vector4f &, Eigen::Vector4f &> (&pcl::getMinMax3D<PointT>), "cloud"_a, "indices"_a, "min_pt"_a, "max_pt"_a);
    m.def("getMinMax3D", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Vector4f &, Eigen::Vector4f &> (&pcl::getMinMax3D<PointT>), "cloud"_a, "indices"_a, "min_pt"_a, "max_pt"_a);
    m.def("getPointsInBox", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Vector4f &, Eigen::Vector4f &, std::vector<int> &> (&pcl::getPointsInBox<PointT>), "cloud"_a, "min_pt"_a, "max_pt"_a, "indices"_a);
}

void defineCommonCommonFunctions(py::module &m) {
    defineCommonCommonFunctions1(m);
    defineCommonCommonFunctions2<pcl::PointXYZ>(m);
    defineCommonCommonFunctions2<pcl::PointXYZI>(m);
    defineCommonCommonFunctions2<pcl::PointXYZL>(m);
    defineCommonCommonFunctions2<pcl::PointXYZRGBA>(m);
    defineCommonCommonFunctions2<pcl::PointXYZRGB>(m);
    defineCommonCommonFunctions2<pcl::PointXYZRGBL>(m);
    defineCommonCommonFunctions2<pcl::PointXYZHSV>(m);
    defineCommonCommonFunctions2<pcl::InterestPoint>(m);
    defineCommonCommonFunctions2<pcl::PointNormal>(m);
    defineCommonCommonFunctions2<pcl::PointXYZRGBNormal>(m);
    defineCommonCommonFunctions2<pcl::PointXYZINormal>(m);
    defineCommonCommonFunctions2<pcl::PointXYZLNormal>(m);
    defineCommonCommonFunctions2<pcl::PointWithRange>(m);
    defineCommonCommonFunctions2<pcl::PointWithViewpoint>(m);
    defineCommonCommonFunctions2<pcl::PointWithScale>(m);
    defineCommonCommonFunctions2<pcl::PointSurfel>(m);
    defineCommonCommonFunctions2<pcl::PointDEM>(m);
}

void defineCommonCommonClasses(py::module &sub_module) {
    defineCommonCommonFunctions(sub_module);
}