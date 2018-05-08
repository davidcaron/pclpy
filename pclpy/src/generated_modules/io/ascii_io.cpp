
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

#include <pcl/io/ascii_io.h>



void defineIoASCIIReader(py::module &m) {
    using Class = pcl::ASCIIReader;
    py::class_<Class, pcl::FileReader, boost::shared_ptr<Class>> cls(m, "ASCIIReader");
    cls.def(py::init<>());
    cls.def("readHeader", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, Eigen::Vector4f &, Eigen::Quaternionf &, int &, int &, unsigned int &, const int> (&Class::readHeader), "file_name"_a, "cloud"_a, "origin"_a, "orientation"_a, "file_version"_a, "data_type"_a, "data_idx"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, Eigen::Vector4f &, Eigen::Quaternionf &, int &, const int> (&Class::read), "file_name"_a, "cloud"_a, "origin"_a, "orientation"_a, "file_version"_a, "offset"_a=0);
    cls.def("setSepChars", &Class::setSepChars, "chars"_a);
    cls.def("setExtension", &Class::setExtension, "ext"_a);
}

void defineIoAsciiIoFunctions(py::module &m) {
}

void defineIoAsciiIoClasses(py::module &sub_module) {
    defineIoASCIIReader(sub_module);
}