
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/io/lzf_image_io.h>

using namespace pcl::io;


void defineIoCameraParameters(py::module &m) {
    using Class = pcl::io::CameraParameters;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "CameraParameters");
    cls.def_readwrite("focal_length_x", &Class::focal_length_x);
    cls.def_readwrite("focal_length_y", &Class::focal_length_y);
    cls.def_readwrite("principal_point_x", &Class::principal_point_x);
    cls.def_readwrite("principal_point_y", &Class::principal_point_y);
}

void defineIoLZFImageReader(py::module &m) {
    using Class = pcl::io::LZFImageReader;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LZFImageReader");
    cls.def(py::init<>());
    cls.def("readParameters", py::overload_cast<const std::string &> (&Class::readParameters), "filename"_a);
    cls.def("setParameters", &Class::setParameters, "parameters"_a);
    cls.def("getParameters", &Class::getParameters);
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
    cls.def("getImageType", &Class::getImageType);
}

void defineIoLZFDepth16ImageReader(py::module &m) {
    using Class = pcl::io::LZFDepth16ImageReader;
    py::class_<Class, pcl::io::LZFImageReader, boost::shared_ptr<Class>> cls(m, "LZFDepth16ImageReader");
    cls.def(py::init<>());
    cls.def("readParameters", py::overload_cast<std::istream &> (&Class::readParameters), "is"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZ> &> (&Class::read<pcl::PointXYZ>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZI> &> (&Class::read<pcl::PointXYZI>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZL> &> (&Class::read<pcl::PointXYZL>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBA> &> (&Class::read<pcl::PointXYZRGBA>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGB> &> (&Class::read<pcl::PointXYZRGB>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBL> &> (&Class::read<pcl::PointXYZRGBL>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZHSV> &> (&Class::read<pcl::PointXYZHSV>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::InterestPoint> &> (&Class::read<pcl::InterestPoint>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointNormal> &> (&Class::read<pcl::PointNormal>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBNormal> &> (&Class::read<pcl::PointXYZRGBNormal>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZINormal> &> (&Class::read<pcl::PointXYZINormal>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZLNormal> &> (&Class::read<pcl::PointXYZLNormal>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithRange> &> (&Class::read<pcl::PointWithRange>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithViewpoint> &> (&Class::read<pcl::PointWithViewpoint>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithScale> &> (&Class::read<pcl::PointWithScale>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointSurfel> &> (&Class::read<pcl::PointSurfel>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointDEM> &> (&Class::read<pcl::PointDEM>), "filename"_a, "cloud"_a);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZ> &, unsigned int> (&Class::readOMP<pcl::PointXYZ>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZI> &, unsigned int> (&Class::readOMP<pcl::PointXYZI>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZL> &, unsigned int> (&Class::readOMP<pcl::PointXYZL>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBA> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBA>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGB> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGB>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBL> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBL>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZHSV> &, unsigned int> (&Class::readOMP<pcl::PointXYZHSV>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::InterestPoint> &, unsigned int> (&Class::readOMP<pcl::InterestPoint>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointNormal> &, unsigned int> (&Class::readOMP<pcl::PointNormal>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBNormal> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBNormal>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZINormal> &, unsigned int> (&Class::readOMP<pcl::PointXYZINormal>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZLNormal> &, unsigned int> (&Class::readOMP<pcl::PointXYZLNormal>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithRange> &, unsigned int> (&Class::readOMP<pcl::PointWithRange>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithViewpoint> &, unsigned int> (&Class::readOMP<pcl::PointWithViewpoint>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithScale> &, unsigned int> (&Class::readOMP<pcl::PointWithScale>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointSurfel> &, unsigned int> (&Class::readOMP<pcl::PointSurfel>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointDEM> &, unsigned int> (&Class::readOMP<pcl::PointDEM>), "filename"_a, "cloud"_a, "num_threads"_a=0);
}

void defineIoLZFImageWriter(py::module &m) {
    using Class = pcl::io::LZFImageWriter;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LZFImageWriter");
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const std::string &> (&Class::write), "data"_a, "width"_a, "height"_a, "filename"_a);
    cls.def("writeParameters", &Class::writeParameters, "parameters"_a, "filename"_a);
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const CameraParameters &, const std::string &, const std::string &> (&Class::write), "data"_a, "width"_a, "height"_a, "parameters"_a, "filename_data"_a, "filename_xml"_a);
    cls.def("writeParameter", &Class::writeParameter, "parameter"_a, "tag"_a, "filename"_a);
}

void defineIoLZFDepth16ImageWriter(py::module &m) {
    using Class = pcl::io::LZFDepth16ImageWriter;
    py::class_<Class, pcl::io::LZFImageWriter, boost::shared_ptr<Class>> cls(m, "LZFDepth16ImageWriter");
    cls.def(py::init<>());
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const std::string &> (&Class::write), "data"_a, "width"_a, "height"_a, "filename"_a);
    cls.def("writeParameters", &Class::writeParameters, "parameters"_a, "filename"_a);
}

void defineIoLZFRGB24ImageReader(py::module &m) {
    using Class = pcl::io::LZFRGB24ImageReader;
    py::class_<Class, pcl::io::LZFImageReader, boost::shared_ptr<Class>> cls(m, "LZFRGB24ImageReader");
    cls.def(py::init<>());
    cls.def("readParameters", py::overload_cast<std::istream &> (&Class::readParameters), "is"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBA> &> (&Class::read<pcl::PointXYZRGBA>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGB> &> (&Class::read<pcl::PointXYZRGB>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBL> &> (&Class::read<pcl::PointXYZRGBL>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBNormal> &> (&Class::read<pcl::PointXYZRGBNormal>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointSurfel> &> (&Class::read<pcl::PointSurfel>), "filename"_a, "cloud"_a);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBA> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBA>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGB> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGB>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBL> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBL>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBNormal> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBNormal>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointSurfel> &, unsigned int> (&Class::readOMP<pcl::PointSurfel>), "filename"_a, "cloud"_a, "num_threads"_a=0);
}

void defineIoLZFBayer8ImageReader(py::module &m) {
    using Class = pcl::io::LZFBayer8ImageReader;
    py::class_<Class, pcl::io::LZFRGB24ImageReader, boost::shared_ptr<Class>> cls(m, "LZFBayer8ImageReader");
    cls.def(py::init<>());
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBA> &> (&Class::read<pcl::PointXYZRGBA>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGB> &> (&Class::read<pcl::PointXYZRGB>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBL> &> (&Class::read<pcl::PointXYZRGBL>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBNormal> &> (&Class::read<pcl::PointXYZRGBNormal>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointSurfel> &> (&Class::read<pcl::PointSurfel>), "filename"_a, "cloud"_a);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBA> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBA>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGB> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGB>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBL> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBL>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBNormal> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBNormal>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointSurfel> &, unsigned int> (&Class::readOMP<pcl::PointSurfel>), "filename"_a, "cloud"_a, "num_threads"_a=0);
}

void defineIoLZFRGB24ImageWriter(py::module &m) {
    using Class = pcl::io::LZFRGB24ImageWriter;
    py::class_<Class, pcl::io::LZFImageWriter, boost::shared_ptr<Class>> cls(m, "LZFRGB24ImageWriter");
    cls.def(py::init<>());
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const std::string &> (&Class::write), "data"_a, "width"_a, "height"_a, "filename"_a);
    cls.def("writeParameters", &Class::writeParameters, "parameters"_a, "filename"_a);
}

void defineIoLZFBayer8ImageWriter(py::module &m) {
    using Class = pcl::io::LZFBayer8ImageWriter;
    py::class_<Class, pcl::io::LZFRGB24ImageWriter, boost::shared_ptr<Class>> cls(m, "LZFBayer8ImageWriter");
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const std::string &> (&Class::write), "data"_a, "width"_a, "height"_a, "filename"_a);
}

void defineIoLZFYUV422ImageReader(py::module &m) {
    using Class = pcl::io::LZFYUV422ImageReader;
    py::class_<Class, pcl::io::LZFRGB24ImageReader, boost::shared_ptr<Class>> cls(m, "LZFYUV422ImageReader");
    cls.def(py::init<>());
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBA> &> (&Class::read<pcl::PointXYZRGBA>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGB> &> (&Class::read<pcl::PointXYZRGB>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBL> &> (&Class::read<pcl::PointXYZRGBL>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBNormal> &> (&Class::read<pcl::PointXYZRGBNormal>), "filename"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointSurfel> &> (&Class::read<pcl::PointSurfel>), "filename"_a, "cloud"_a);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBA> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBA>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGB> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGB>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBL> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBL>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBNormal> &, unsigned int> (&Class::readOMP<pcl::PointXYZRGBNormal>), "filename"_a, "cloud"_a, "num_threads"_a=0);
    cls.def("readOMP", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointSurfel> &, unsigned int> (&Class::readOMP<pcl::PointSurfel>), "filename"_a, "cloud"_a, "num_threads"_a=0);
}

void defineIoLZFYUV422ImageWriter(py::module &m) {
    using Class = pcl::io::LZFYUV422ImageWriter;
    py::class_<Class, pcl::io::LZFRGB24ImageWriter, boost::shared_ptr<Class>> cls(m, "LZFYUV422ImageWriter");
    cls.def("write", py::overload_cast<const char *, uint32_t, uint32_t, const std::string &> (&Class::write), "data"_a, "width"_a, "height"_a, "filename"_a);
}

void defineIoLzfImageIoFunctions(py::module &m) {
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