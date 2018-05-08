
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

#include <pcl/features/pfh_tools.h>



void defineFeaturesPfhToolsFunctions1(py::module &m) {
    m.def("computePairFeatures", py::overload_cast<const Eigen::Vector4f &, const Eigen::Vector4f &, const Eigen::Vector4f &, const Eigen::Vector4f &, float &, float &, float &, float &> (&pcl::computePairFeatures), "p1"_a, "n1"_a, "p2"_a, "n2"_a, "f1"_a, "f2"_a, "f3"_a, "f4"_a);
    m.def("computeRGBPairFeatures", py::overload_cast<const Eigen::Vector4f &, const Eigen::Vector4f &, const Eigen::Vector4i &, const Eigen::Vector4f &, const Eigen::Vector4f &, const Eigen::Vector4i &, float &, float &, float &, float &, float &, float &, float &> (&pcl::computeRGBPairFeatures), "p1"_a, "n1"_a, "colors1"_a, "p2"_a, "n2"_a, "colors2"_a, "f1"_a, "f2"_a, "f3"_a, "f4"_a, "f5"_a, "f6"_a, "f7"_a);
}

void defineFeaturesPfhToolsFunctions(py::module &m) {
    defineFeaturesPfhToolsFunctions1(m);
}

void defineFeaturesPfhToolsClasses(py::module &sub_module) {
    defineFeaturesPfhToolsFunctions(sub_module);
}