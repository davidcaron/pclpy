
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/io/ply/ply.h>



void defineIoPlyFunctions(py::module &m) {
}

void defineIoPlyClasses(py::module &sub_module) {
    py::enum_<pcl::io::ply::format>(sub_module, "format")
        .value("ascii_format", pcl::io::ply::format::ascii_format)
        .value("binary_little_endian_format", pcl::io::ply::format::binary_little_endian_format)
        .value("binary_big_endian_format", pcl::io::ply::format::binary_big_endian_format)
        .value("unknown", pcl::io::ply::format::unknown)
        .export_values();
}