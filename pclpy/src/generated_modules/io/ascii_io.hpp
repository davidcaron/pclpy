
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/ascii_io.h>



void defineIoASCIIReader(py::module &m) {
    using Class = ASCIIReader;
    py::class_<Class, FileReader, boost::shared_ptr<Class>> cls(m, "ASCIIReader");
    cls.def(py::init<>());
    cls.def("set_sep_chars", &Class::setSepChars);
    cls.def("set_extension", &Class::setExtension);
    cls.def("read_header", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, Eigen::Vector4f &, Eigen::Quaternionf &, int &, int &, unsigned int &, const int> (&Class::readHeader));
    cls.def("read", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, Eigen::Vector4f &, Eigen::Quaternionf &, int &, const int> (&Class::read));
}

void defineIoAsciiIoClasses(py::module &sub_module) {
    defineIoASCIIReader(sub_module);
}