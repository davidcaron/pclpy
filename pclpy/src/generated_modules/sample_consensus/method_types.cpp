
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

#include <pcl/sample_consensus/method_types.h>



void defineSampleConsensusMethodTypesFunctions(py::module &m) {
}

void defineSampleConsensusMethodTypesClasses(py::module &sub_module) {
    sub_module.attr("SAC_LMEDS") = 1;
    sub_module.attr("SAC_MLESAC") = 5;
    sub_module.attr("SAC_MSAC") = 2;
    sub_module.attr("SAC_PROSAC") = 6;
    sub_module.attr("SAC_RANSAC") = 0;
    sub_module.attr("SAC_RMSAC") = 4;
    sub_module.attr("SAC_RRANSAC") = 3;
}