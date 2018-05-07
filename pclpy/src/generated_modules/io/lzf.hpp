
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/io/lzf.h>



void defineIoLzfFunctions1(py::module &m) {
    m.def("lzfCompress", py::overload_cast<const void * const, unsigned int, void *, unsigned int> (&pcl::lzfCompress), "in_data"_a, "in_len"_a, "out_data"_a, "out_len"_a);
    m.def("lzfDecompress", py::overload_cast<const void * const, unsigned int, void *, unsigned int> (&pcl::lzfDecompress), "in_data"_a, "in_len"_a, "out_data"_a, "out_len"_a);
}

void defineIoLzfFunctions(py::module &m) {
    defineIoLzfFunctions1(m);
}

void defineIoLzfClasses(py::module &sub_module) {
    defineIoLzfFunctions(sub_module);
}