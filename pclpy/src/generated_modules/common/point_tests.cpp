
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

#include <pcl/common/point_tests.h>



template <typename PointT>
void defineCommonPointTestsFunctions1(py::module &m) {
    m.def("isFinite", py::overload_cast<const PointT &> (&pcl::isFinite<PointT>), "pt"_a);
    m.def("isFinite", py::overload_cast<const Eigen::internal::workaround_msvc_stl_support<PointT> &> (&pcl::isFinite<PointT>), "pt"_a);
}

void defineCommonPointTestsFunctions(py::module &m) {
    defineCommonPointTestsFunctions1<pcl::PointXYZ>(m);
    defineCommonPointTestsFunctions1<pcl::PointXYZI>(m);
    defineCommonPointTestsFunctions1<pcl::PointXYZL>(m);
    defineCommonPointTestsFunctions1<pcl::PointXYZRGBA>(m);
    defineCommonPointTestsFunctions1<pcl::PointXYZRGB>(m);
    defineCommonPointTestsFunctions1<pcl::PointXYZRGBL>(m);
    defineCommonPointTestsFunctions1<pcl::PointXYZHSV>(m);
    defineCommonPointTestsFunctions1<pcl::InterestPoint>(m);
    defineCommonPointTestsFunctions1<pcl::PointNormal>(m);
    defineCommonPointTestsFunctions1<pcl::PointXYZRGBNormal>(m);
    defineCommonPointTestsFunctions1<pcl::PointXYZINormal>(m);
    defineCommonPointTestsFunctions1<pcl::PointXYZLNormal>(m);
    defineCommonPointTestsFunctions1<pcl::PointWithRange>(m);
    defineCommonPointTestsFunctions1<pcl::PointWithViewpoint>(m);
    defineCommonPointTestsFunctions1<pcl::PointWithScale>(m);
    defineCommonPointTestsFunctions1<pcl::PointSurfel>(m);
    defineCommonPointTestsFunctions1<pcl::PointDEM>(m);
}

void defineCommonPointTestsClasses(py::module &sub_module) {
    defineCommonPointTestsFunctions(sub_module);
}