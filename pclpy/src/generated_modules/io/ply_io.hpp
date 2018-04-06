
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/ply_io.h>



void defineIoPLYReader(py::module &m) {
    using Class = PLYReader;
    py::class_<Class, FileReader, boost::shared_ptr<Class>> cls(m, "PLYReader");
    cls.def(py::init<>());
    cls.def("read_header", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, Eigen::Vector4f &, Eigen::Quaternionf &, int &, int &, unsigned int &, const int> (&Class::readHeader));
    // Operators not implemented (operator=);
}

void defineIoPLYWriter(py::module &m) {
    using Class = PLYWriter;
    py::class_<Class, FileWriter, boost::shared_ptr<Class>> cls(m, "PLYWriter");
    cls.def(py::init<>());
    cls.def("write_ascii", py::overload_cast<const std::string &, const pcl::PCLPointCloud2 &, const Eigen::Vector4f &, const Eigen::Quaternionf &, int, bool> (&Class::writeASCII));
    cls.def("write_binary", py::overload_cast<const std::string &, const pcl::PCLPointCloud2 &, const Eigen::Vector4f &, const Eigen::Quaternionf &, bool> (&Class::writeBinary));
    cls.def("generate_header_binary", &Class::generateHeaderBinary);
    cls.def("generate_header_ascii", &Class::generateHeaderASCII);
}

void defineIoPlyIoClasses(py::module &sub_module) {
    defineIoPLYReader(sub_module);
    defineIoPLYWriter(sub_module);
}