
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/lzf_image_io.h>

using namespace pcl::io;


void defineIoCameraParameters(py::module &m) {
    using Class = io::CameraParameters;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "CameraParameters");
    cls.def_readonly("focal_length_x", &Class::focal_length_x);
    cls.def_readonly("focal_length_y", &Class::focal_length_y);
    cls.def_readonly("principal_point_x", &Class::principal_point_x);
    cls.def_readonly("principal_point_y", &Class::principal_point_y);
}

void defineIoLZFImageReader(py::module &m) {
    using Class = io::LZFImageReader;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LZFImageReader");
    cls.def(py::init<>());
    cls.def_property("parameters", &Class::getParameters, &Class::setParameters);
    cls.def("read_parameters", py::overload_cast<const std::string &> (&Class::readParameters));
}

void defineIoLZFDepth16ImageReader(py::module &m) {
    using Class = io::LZFDepth16ImageReader;
    py::class_<Class, LZFImageReader, boost::shared_ptr<Class>> cls(m, "LZFDepth16ImageReader");
    cls.def(py::init<>());
    cls.def("read_parameters", py::overload_cast<std::istream &> (&Class::readParameters));
}

void defineIoLZFImageWriter(py::module &m) {
    using Class = io::LZFImageWriter;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LZFImageWriter");
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const std::string &> (&Class::write));
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const CameraParameters &, const std::string &, const std::string &> (&Class::write));
    cls.def("write_parameters", &Class::writeParameters);
    cls.def("write_parameter", &Class::writeParameter);
}

void defineIoLZFDepth16ImageWriter(py::module &m) {
    using Class = io::LZFDepth16ImageWriter;
    py::class_<Class, LZFImageWriter, boost::shared_ptr<Class>> cls(m, "LZFDepth16ImageWriter");
    cls.def(py::init<>());
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const std::string &> (&Class::write));
    cls.def("write_parameters", &Class::writeParameters);
}

void defineIoLZFRGB24ImageReader(py::module &m) {
    using Class = io::LZFRGB24ImageReader;
    py::class_<Class, LZFImageReader, boost::shared_ptr<Class>> cls(m, "LZFRGB24ImageReader");
    cls.def(py::init<>());
    cls.def("read_parameters", py::overload_cast<std::istream &> (&Class::readParameters));
}

void defineIoLZFBayer8ImageReader(py::module &m) {
    using Class = io::LZFBayer8ImageReader;
    py::class_<Class, LZFRGB24ImageReader, boost::shared_ptr<Class>> cls(m, "LZFBayer8ImageReader");
    cls.def(py::init<>());
}

void defineIoLZFRGB24ImageWriter(py::module &m) {
    using Class = io::LZFRGB24ImageWriter;
    py::class_<Class, LZFImageWriter, boost::shared_ptr<Class>> cls(m, "LZFRGB24ImageWriter");
    cls.def(py::init<>());
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const std::string &> (&Class::write));
    cls.def("write_parameters", &Class::writeParameters);
}

void defineIoLZFBayer8ImageWriter(py::module &m) {
    using Class = io::LZFBayer8ImageWriter;
    py::class_<Class, LZFRGB24ImageWriter, boost::shared_ptr<Class>> cls(m, "LZFBayer8ImageWriter");
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const std::string &> (&Class::write));
}

void defineIoLZFYUV422ImageReader(py::module &m) {
    using Class = io::LZFYUV422ImageReader;
    py::class_<Class, LZFRGB24ImageReader, boost::shared_ptr<Class>> cls(m, "LZFYUV422ImageReader");
    cls.def(py::init<>());
}

void defineIoLZFYUV422ImageWriter(py::module &m) {
    using Class = io::LZFYUV422ImageWriter;
    py::class_<Class, LZFRGB24ImageWriter, boost::shared_ptr<Class>> cls(m, "LZFYUV422ImageWriter");
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const std::string &> (&Class::write));
}

void defineIoLzfImageIoClasses(py::module &sub_module) {
    defineIoCameraParameters(sub_module);
    defineIoLZFImageReader(sub_module);
    defineIoLZFDepth16ImageReader(sub_module);
    defineIoLZFImageWriter(sub_module);
    defineIoLZFDepth16ImageWriter(sub_module);
    defineIoLZFRGB24ImageReader(sub_module);
    defineIoLZFBayer8ImageReader(sub_module);
    defineIoLZFRGB24ImageWriter(sub_module);
    defineIoLZFBayer8ImageWriter(sub_module);
    defineIoLZFYUV422ImageReader(sub_module);
    defineIoLZFYUV422ImageWriter(sub_module);
}