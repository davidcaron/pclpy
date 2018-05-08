
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

#include <pcl/common/projection_matrix.h>



void defineCommonProjectionMatrixFunctions1(py::module &m) {
    m.def("getCameraMatrixFromProjectionMatrix", py::overload_cast<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor> &, Eigen::Matrix3f &> (&pcl::getCameraMatrixFromProjectionMatrix), "projection_matrix"_a, "camera_matrix"_a);
}

template<typename PointT>
void defineCommonProjectionMatrixFunctions2(py::module &m) {
    m.def("estimateProjectionMatrix", py::overload_cast<pcl::PointCloud<PointT>::ConstPtr, Eigen::Matrix<float, 3, 4, Eigen::RowMajor> &, const std::vector<int> &> (&pcl::estimateProjectionMatrix<PointT>), "cloud"_a, "projection_matrix"_a, "indices"_a=std::vector<int>());
}

void defineCommonProjectionMatrixFunctions(py::module &m) {
    defineCommonProjectionMatrixFunctions1(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointXYZ>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointXYZI>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointXYZL>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointXYZRGBA>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointXYZRGB>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointXYZRGBL>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointXYZHSV>(m);
    defineCommonProjectionMatrixFunctions2<pcl::InterestPoint>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointNormal>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointXYZRGBNormal>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointXYZINormal>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointXYZLNormal>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointWithRange>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointWithViewpoint>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointWithScale>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointSurfel>(m);
    defineCommonProjectionMatrixFunctions2<pcl::PointDEM>(m);
}

void defineCommonProjectionMatrixClasses(py::module &sub_module) {
    defineCommonProjectionMatrixFunctions(sub_module);
}