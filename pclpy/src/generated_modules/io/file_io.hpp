
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/io/file_io.h>



void defineIoFileReader(py::module &m) {
    using Class = pcl::FileReader;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FileReader");
    cls.def("readHeader", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, Eigen::Vector4f &, Eigen::Quaternionf &, int &, int &, unsigned int &, const int> (&Class::readHeader), "file_name"_a, "cloud"_a, "origin"_a, "orientation"_a, "file_version"_a, "data_type"_a, "data_idx"_a, "offset"_a=0);
    // Non templated function disambiguation not implemented (read);
    // Non templated function disambiguation not implemented (read);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZ> &, const int> (&Class::read<pcl::PointXYZ>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZI> &, const int> (&Class::read<pcl::PointXYZI>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZL> &, const int> (&Class::read<pcl::PointXYZL>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::Label> &, const int> (&Class::read<pcl::Label>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBA> &, const int> (&Class::read<pcl::PointXYZRGBA>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGB> &, const int> (&Class::read<pcl::PointXYZRGB>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBL> &, const int> (&Class::read<pcl::PointXYZRGBL>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZHSV> &, const int> (&Class::read<pcl::PointXYZHSV>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXY> &, const int> (&Class::read<pcl::PointXY>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::InterestPoint> &, const int> (&Class::read<pcl::InterestPoint>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::Axis> &, const int> (&Class::read<pcl::Axis>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::Normal> &, const int> (&Class::read<pcl::Normal>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointNormal> &, const int> (&Class::read<pcl::PointNormal>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBNormal> &, const int> (&Class::read<pcl::PointXYZRGBNormal>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZINormal> &, const int> (&Class::read<pcl::PointXYZINormal>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZLNormal> &, const int> (&Class::read<pcl::PointXYZLNormal>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithRange> &, const int> (&Class::read<pcl::PointWithRange>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithViewpoint> &, const int> (&Class::read<pcl::PointWithViewpoint>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::MomentInvariants> &, const int> (&Class::read<pcl::MomentInvariants>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PrincipalRadiiRSD> &, const int> (&Class::read<pcl::PrincipalRadiiRSD>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::Boundary> &, const int> (&Class::read<pcl::Boundary>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PrincipalCurvatures> &, const int> (&Class::read<pcl::PrincipalCurvatures>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PFHSignature125> &, const int> (&Class::read<pcl::PFHSignature125>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PFHRGBSignature250> &, const int> (&Class::read<pcl::PFHRGBSignature250>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PPFSignature> &, const int> (&Class::read<pcl::PPFSignature>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::CPPFSignature> &, const int> (&Class::read<pcl::CPPFSignature>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PPFRGBSignature> &, const int> (&Class::read<pcl::PPFRGBSignature>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::NormalBasedSignature12> &, const int> (&Class::read<pcl::NormalBasedSignature12>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::FPFHSignature33> &, const int> (&Class::read<pcl::FPFHSignature33>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::VFHSignature308> &, const int> (&Class::read<pcl::VFHSignature308>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::GRSDSignature21> &, const int> (&Class::read<pcl::GRSDSignature21>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::ESFSignature640> &, const int> (&Class::read<pcl::ESFSignature640>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::BRISKSignature512> &, const int> (&Class::read<pcl::BRISKSignature512>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::Narf36> &, const int> (&Class::read<pcl::Narf36>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::IntensityGradient> &, const int> (&Class::read<pcl::IntensityGradient>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithScale> &, const int> (&Class::read<pcl::PointWithScale>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointSurfel> &, const int> (&Class::read<pcl::PointSurfel>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::ShapeContext1980> &, const int> (&Class::read<pcl::ShapeContext1980>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::UniqueShapeContext1960> &, const int> (&Class::read<pcl::UniqueShapeContext1960>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::SHOT352> &, const int> (&Class::read<pcl::SHOT352>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::SHOT1344> &, const int> (&Class::read<pcl::SHOT1344>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointUV> &, const int> (&Class::read<pcl::PointUV>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::ReferenceFrame> &, const int> (&Class::read<pcl::ReferenceFrame>), "file_name"_a, "cloud"_a, "offset"_a=0);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointDEM> &, const int> (&Class::read<pcl::PointDEM>), "file_name"_a, "cloud"_a, "offset"_a=0);
}

void defineIoFileWriter(py::module &m) {
    using Class = pcl::FileWriter;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FileWriter");
    // Non templated function disambiguation not implemented (write);
    // Non templated function disambiguation not implemented (write);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZ> &, const bool> (&Class::write<pcl::PointXYZ>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZI> &, const bool> (&Class::write<pcl::PointXYZI>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZL> &, const bool> (&Class::write<pcl::PointXYZL>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::Label> &, const bool> (&Class::write<pcl::Label>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZRGBA> &, const bool> (&Class::write<pcl::PointXYZRGBA>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZRGB> &, const bool> (&Class::write<pcl::PointXYZRGB>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZRGBL> &, const bool> (&Class::write<pcl::PointXYZRGBL>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZHSV> &, const bool> (&Class::write<pcl::PointXYZHSV>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXY> &, const bool> (&Class::write<pcl::PointXY>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::InterestPoint> &, const bool> (&Class::write<pcl::InterestPoint>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::Axis> &, const bool> (&Class::write<pcl::Axis>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::Normal> &, const bool> (&Class::write<pcl::Normal>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointNormal> &, const bool> (&Class::write<pcl::PointNormal>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZRGBNormal> &, const bool> (&Class::write<pcl::PointXYZRGBNormal>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZINormal> &, const bool> (&Class::write<pcl::PointXYZINormal>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZLNormal> &, const bool> (&Class::write<pcl::PointXYZLNormal>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointWithRange> &, const bool> (&Class::write<pcl::PointWithRange>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointWithViewpoint> &, const bool> (&Class::write<pcl::PointWithViewpoint>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::MomentInvariants> &, const bool> (&Class::write<pcl::MomentInvariants>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PrincipalRadiiRSD> &, const bool> (&Class::write<pcl::PrincipalRadiiRSD>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::Boundary> &, const bool> (&Class::write<pcl::Boundary>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PrincipalCurvatures> &, const bool> (&Class::write<pcl::PrincipalCurvatures>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PFHSignature125> &, const bool> (&Class::write<pcl::PFHSignature125>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PFHRGBSignature250> &, const bool> (&Class::write<pcl::PFHRGBSignature250>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PPFSignature> &, const bool> (&Class::write<pcl::PPFSignature>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::CPPFSignature> &, const bool> (&Class::write<pcl::CPPFSignature>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PPFRGBSignature> &, const bool> (&Class::write<pcl::PPFRGBSignature>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::NormalBasedSignature12> &, const bool> (&Class::write<pcl::NormalBasedSignature12>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::FPFHSignature33> &, const bool> (&Class::write<pcl::FPFHSignature33>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::VFHSignature308> &, const bool> (&Class::write<pcl::VFHSignature308>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::GRSDSignature21> &, const bool> (&Class::write<pcl::GRSDSignature21>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::ESFSignature640> &, const bool> (&Class::write<pcl::ESFSignature640>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::BRISKSignature512> &, const bool> (&Class::write<pcl::BRISKSignature512>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::Narf36> &, const bool> (&Class::write<pcl::Narf36>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::IntensityGradient> &, const bool> (&Class::write<pcl::IntensityGradient>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointWithScale> &, const bool> (&Class::write<pcl::PointWithScale>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointSurfel> &, const bool> (&Class::write<pcl::PointSurfel>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::ShapeContext1980> &, const bool> (&Class::write<pcl::ShapeContext1980>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::UniqueShapeContext1960> &, const bool> (&Class::write<pcl::UniqueShapeContext1960>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::SHOT352> &, const bool> (&Class::write<pcl::SHOT352>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::SHOT1344> &, const bool> (&Class::write<pcl::SHOT1344>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointUV> &, const bool> (&Class::write<pcl::PointUV>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::ReferenceFrame> &, const bool> (&Class::write<pcl::ReferenceFrame>), "file_name"_a, "cloud"_a, "binary"_a=false);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointDEM> &, const bool> (&Class::write<pcl::PointDEM>), "file_name"_a, "cloud"_a, "binary"_a=false);
}

void defineIoFileIoFunctions(py::module &m) {
}

void defineIoFileIoClasses(py::module &sub_module) {
    defineIoFileReader(sub_module);
    defineIoFileWriter(sub_module);
    defineIoFileIoFunctions(sub_module);
}