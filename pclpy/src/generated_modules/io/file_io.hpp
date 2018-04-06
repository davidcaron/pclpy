
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/file_io.h>



void defineIoFileReader(py::module &m) {
    using Class = FileReader;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FileReader");
    cls.def("read_header", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, Eigen::Vector4f &, Eigen::Quaternionf &, int &, int &, unsigned int &, const int> (&Class::readHeader));
}

void defineIoFileWriter(py::module &m) {
    using Class = FileWriter;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FileWriter");
}

void defineIoFileIoClasses(py::module &sub_module) {
    defineIoFileReader(sub_module);
    defineIoFileWriter(sub_module);
}