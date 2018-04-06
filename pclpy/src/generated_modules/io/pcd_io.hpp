
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/pcd_io.h>



void defineIoPCDReader(py::module &m) {
    using Class = PCDReader;
    py::class_<Class, FileReader, boost::shared_ptr<Class>> cls(m, "PCDReader");
    cls.def(py::init<>());
    cls.def("read_header", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, Eigen::Vector4f &, Eigen::Quaternionf &, int &, int &, unsigned int &, const int> (&Class::readHeader));
    cls.def("read_header", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, const int> (&Class::readHeader));
}

void defineIoPCDWriter(py::module &m) {
    using Class = PCDWriter;
    py::class_<Class, FileWriter, boost::shared_ptr<Class>> cls(m, "PCDWriter");
    cls.def(py::init<>());
    cls.def("set_map_synchronization", &Class::setMapSynchronization);
    cls.def("generate_header_binary", &Class::generateHeaderBinary);
    cls.def("generate_header_binary_compressed", &Class::generateHeaderBinaryCompressed);
    cls.def("generate_header_ascii", &Class::generateHeaderASCII);
}

void defineIoPcdIoClasses(py::module &sub_module) {
    defineIoPCDReader(sub_module);
    defineIoPCDWriter(sub_module);
}