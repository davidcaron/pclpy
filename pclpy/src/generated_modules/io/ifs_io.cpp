
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/io/ifs_io.h>



void defineIoIFSReader(py::module &m) {
    using Class = pcl::IFSReader;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "IFSReader");
    cls.def(py::init<>());
    cls.def("readHeader", py::overload_cast<const std::string &, pcl::PCLPointCloud2 &, int &, unsigned int &> (&Class::readHeader), "file_name"_a, "cloud"_a, "ifs_version"_a, "data_idx"_a);
    // Non templated function disambiguation not implemented (read);
    // Non templated function disambiguation not implemented (read);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZ> &> (&Class::read<pcl::PointXYZ>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZI> &> (&Class::read<pcl::PointXYZI>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZL> &> (&Class::read<pcl::PointXYZL>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::Label> &> (&Class::read<pcl::Label>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBA> &> (&Class::read<pcl::PointXYZRGBA>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGB> &> (&Class::read<pcl::PointXYZRGB>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBL> &> (&Class::read<pcl::PointXYZRGBL>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZHSV> &> (&Class::read<pcl::PointXYZHSV>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXY> &> (&Class::read<pcl::PointXY>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::InterestPoint> &> (&Class::read<pcl::InterestPoint>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::Axis> &> (&Class::read<pcl::Axis>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::Normal> &> (&Class::read<pcl::Normal>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointNormal> &> (&Class::read<pcl::PointNormal>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZRGBNormal> &> (&Class::read<pcl::PointXYZRGBNormal>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZINormal> &> (&Class::read<pcl::PointXYZINormal>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointXYZLNormal> &> (&Class::read<pcl::PointXYZLNormal>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithRange> &> (&Class::read<pcl::PointWithRange>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithViewpoint> &> (&Class::read<pcl::PointWithViewpoint>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::MomentInvariants> &> (&Class::read<pcl::MomentInvariants>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PrincipalRadiiRSD> &> (&Class::read<pcl::PrincipalRadiiRSD>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::Boundary> &> (&Class::read<pcl::Boundary>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PrincipalCurvatures> &> (&Class::read<pcl::PrincipalCurvatures>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PFHSignature125> &> (&Class::read<pcl::PFHSignature125>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PFHRGBSignature250> &> (&Class::read<pcl::PFHRGBSignature250>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PPFSignature> &> (&Class::read<pcl::PPFSignature>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::CPPFSignature> &> (&Class::read<pcl::CPPFSignature>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PPFRGBSignature> &> (&Class::read<pcl::PPFRGBSignature>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::NormalBasedSignature12> &> (&Class::read<pcl::NormalBasedSignature12>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::FPFHSignature33> &> (&Class::read<pcl::FPFHSignature33>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::VFHSignature308> &> (&Class::read<pcl::VFHSignature308>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::GRSDSignature21> &> (&Class::read<pcl::GRSDSignature21>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::ESFSignature640> &> (&Class::read<pcl::ESFSignature640>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::BRISKSignature512> &> (&Class::read<pcl::BRISKSignature512>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::Narf36> &> (&Class::read<pcl::Narf36>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::IntensityGradient> &> (&Class::read<pcl::IntensityGradient>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointWithScale> &> (&Class::read<pcl::PointWithScale>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointSurfel> &> (&Class::read<pcl::PointSurfel>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::ShapeContext1980> &> (&Class::read<pcl::ShapeContext1980>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::UniqueShapeContext1960> &> (&Class::read<pcl::UniqueShapeContext1960>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::SHOT352> &> (&Class::read<pcl::SHOT352>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::SHOT1344> &> (&Class::read<pcl::SHOT1344>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointUV> &> (&Class::read<pcl::PointUV>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::ReferenceFrame> &> (&Class::read<pcl::ReferenceFrame>), "file_name"_a, "cloud"_a);
    cls.def("read", py::overload_cast<const std::string &, pcl::PointCloud<pcl::PointDEM> &> (&Class::read<pcl::PointDEM>), "file_name"_a, "cloud"_a);
}

void defineIoIFSWriter(py::module &m) {
    using Class = pcl::IFSWriter;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "IFSWriter");
    cls.def(py::init<>());
    // Non templated function disambiguation not implemented (write);
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZ> &, const std::string &> (&Class::write<pcl::PointXYZ>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZI> &, const std::string &> (&Class::write<pcl::PointXYZI>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZL> &, const std::string &> (&Class::write<pcl::PointXYZL>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::Label> &, const std::string &> (&Class::write<pcl::Label>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZRGBA> &, const std::string &> (&Class::write<pcl::PointXYZRGBA>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZRGB> &, const std::string &> (&Class::write<pcl::PointXYZRGB>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZRGBL> &, const std::string &> (&Class::write<pcl::PointXYZRGBL>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZHSV> &, const std::string &> (&Class::write<pcl::PointXYZHSV>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXY> &, const std::string &> (&Class::write<pcl::PointXY>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::InterestPoint> &, const std::string &> (&Class::write<pcl::InterestPoint>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::Axis> &, const std::string &> (&Class::write<pcl::Axis>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::Normal> &, const std::string &> (&Class::write<pcl::Normal>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointNormal> &, const std::string &> (&Class::write<pcl::PointNormal>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZRGBNormal> &, const std::string &> (&Class::write<pcl::PointXYZRGBNormal>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZINormal> &, const std::string &> (&Class::write<pcl::PointXYZINormal>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointXYZLNormal> &, const std::string &> (&Class::write<pcl::PointXYZLNormal>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointWithRange> &, const std::string &> (&Class::write<pcl::PointWithRange>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointWithViewpoint> &, const std::string &> (&Class::write<pcl::PointWithViewpoint>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::MomentInvariants> &, const std::string &> (&Class::write<pcl::MomentInvariants>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PrincipalRadiiRSD> &, const std::string &> (&Class::write<pcl::PrincipalRadiiRSD>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::Boundary> &, const std::string &> (&Class::write<pcl::Boundary>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PrincipalCurvatures> &, const std::string &> (&Class::write<pcl::PrincipalCurvatures>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PFHSignature125> &, const std::string &> (&Class::write<pcl::PFHSignature125>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PFHRGBSignature250> &, const std::string &> (&Class::write<pcl::PFHRGBSignature250>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PPFSignature> &, const std::string &> (&Class::write<pcl::PPFSignature>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::CPPFSignature> &, const std::string &> (&Class::write<pcl::CPPFSignature>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PPFRGBSignature> &, const std::string &> (&Class::write<pcl::PPFRGBSignature>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::NormalBasedSignature12> &, const std::string &> (&Class::write<pcl::NormalBasedSignature12>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::FPFHSignature33> &, const std::string &> (&Class::write<pcl::FPFHSignature33>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::VFHSignature308> &, const std::string &> (&Class::write<pcl::VFHSignature308>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::GRSDSignature21> &, const std::string &> (&Class::write<pcl::GRSDSignature21>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::ESFSignature640> &, const std::string &> (&Class::write<pcl::ESFSignature640>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::BRISKSignature512> &, const std::string &> (&Class::write<pcl::BRISKSignature512>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::Narf36> &, const std::string &> (&Class::write<pcl::Narf36>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::IntensityGradient> &, const std::string &> (&Class::write<pcl::IntensityGradient>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointWithScale> &, const std::string &> (&Class::write<pcl::PointWithScale>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointSurfel> &, const std::string &> (&Class::write<pcl::PointSurfel>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::ShapeContext1980> &, const std::string &> (&Class::write<pcl::ShapeContext1980>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::UniqueShapeContext1960> &, const std::string &> (&Class::write<pcl::UniqueShapeContext1960>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::SHOT352> &, const std::string &> (&Class::write<pcl::SHOT352>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::SHOT1344> &, const std::string &> (&Class::write<pcl::SHOT1344>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointUV> &, const std::string &> (&Class::write<pcl::PointUV>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::ReferenceFrame> &, const std::string &> (&Class::write<pcl::ReferenceFrame>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
    cls.def("write", py::overload_cast<const std::string &, const pcl::PointCloud<pcl::PointDEM> &, const std::string &> (&Class::write<pcl::PointDEM>), "file_name"_a, "cloud"_a, "cloud_name"_a="cloud");
}

void defineIoIfsIoFunctions(py::module &m) {
}

void defineIoIfsIoClasses(py::module &sub_module) {
    defineIoIFSReader(sub_module);
    defineIoIFSWriter(sub_module);
}