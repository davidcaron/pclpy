
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/ifs_io.h>



void defineIoIFSReader(py::module &m) {
    using Class = IFSReader;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "IFSReader");
    cls.def(py::init<>());
    cls.def("read_header", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, int &, unsigned int &> (&Class::readHeader));
}

void defineIoIFSWriter(py::module &m) {
    using Class = IFSWriter;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "IFSWriter");
    cls.def(py::init<>());
}

void defineIoIfsIoClasses(py::module &sub_module) {
    defineIoIFSReader(sub_module);
    defineIoIFSWriter(sub_module);
}