
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

#include <pcl/filters/morphological_filter.h>



template <typename PointT>
void defineFiltersMorphologicalFilterFunctions1(py::module &m) {
    m.def("applyMorphologicalOperator", py::overload_cast<const pcl::PointCloud<PointT>::ConstPtr &, float, const int, pcl::PointCloud<PointT> &> (&pcl::applyMorphologicalOperator<PointT>), "cloud_in"_a, "resolution"_a, "morphological_operator"_a, "cloud_out"_a);
}

void defineFiltersMorphologicalFilterFunctions(py::module &m) {
    defineFiltersMorphologicalFilterFunctions1<pcl::PointXYZ>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointXYZI>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointXYZL>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointXYZRGBA>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointXYZRGB>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointXYZRGBL>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointXYZHSV>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::InterestPoint>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointNormal>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointXYZRGBNormal>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointXYZINormal>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointXYZLNormal>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointWithRange>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointWithViewpoint>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointWithScale>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointSurfel>(m);
    defineFiltersMorphologicalFilterFunctions1<pcl::PointDEM>(m);
}

void defineFiltersMorphologicalFilterClasses(py::module &sub_module) {
    defineFiltersMorphologicalFilterFunctions(sub_module);
    py::enum_<pcl::MorphologicalOperators>(sub_module, "MorphologicalOperators")
        .value("MORPH_OPEN", pcl::MorphologicalOperators::MORPH_OPEN)
        .value("MORPH_CLOSE", pcl::MorphologicalOperators::MORPH_CLOSE)
        .value("MORPH_DILATE", pcl::MorphologicalOperators::MORPH_DILATE)
        .value("MORPH_ERODE", pcl::MorphologicalOperators::MORPH_ERODE)
        .export_values();
}