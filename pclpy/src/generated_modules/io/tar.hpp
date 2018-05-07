
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/io/tar.h>

using namespace pcl::io;


void defineIoTARHeader(py::module &m) {
    using Class = pcl::io::TARHeader;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "TARHeader");
    cls.def_readonly("file_name", &Class::file_name);
    cls.def_readonly("file_mode", &Class::file_mode);
    cls.def_readonly("uid", &Class::uid);
    cls.def_readonly("gid", &Class::gid);
    cls.def_readonly("file_size", &Class::file_size);
    cls.def_readonly("mtime", &Class::mtime);
    cls.def_readonly("chksum", &Class::chksum);
    cls.def_readonly("file_type", &Class::file_type);
    cls.def_readonly("link_file_name", &Class::link_file_name);
    cls.def_readonly("ustar", &Class::ustar);
    cls.def_readonly("ustar_version", &Class::ustar_version);
    cls.def_readonly("uname", &Class::uname);
    cls.def_readonly("gname", &Class::gname);
    cls.def_readonly("dev_major", &Class::dev_major);
    cls.def_readonly("dev_minor", &Class::dev_minor);
    cls.def_readonly("file_name_prefix", &Class::file_name_prefix);
    cls.def_readonly("_padding", &Class::_padding);
    cls.def("getFileSize", &Class::getFileSize);
}

void defineIoTarFunctions(py::module &m) {
}

void defineIoTarClasses(py::module &sub_module) {
    defineIoTARHeader(sub_module);
}