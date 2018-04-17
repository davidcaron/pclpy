
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/shared_ptr.hpp>

namespace py = pybind11;
using namespace pybind11::literals;


void definePointTypes(py::module &m) {
    py::module m_pts = m.def_submodule("point_types", "Submodule for point types");

    // PointXYZ
    using Class = pcl::PointXYZ;
    py::class_<Class, boost::shared_ptr<Class>> cls(m_pts, "PointXYZ");
    cls.def(py::init<>());
    cls.def_readwrite("x", &Class::x);
    cls.def_readwrite("y", &Class::y);
    cls.def_readwrite("z", &Class::z);

    using Class = pcl::PointXYZI;
    py::class_<Class, boost::shared_ptr<Class>> cls(m_pts, "PointXYZI");
    cls.def(py::init<>());
    cls.def_readwrite("x", &Class::x);
    cls.def_readwrite("y", &Class::y);
    cls.def_readwrite("z", &Class::z);
    cls.def_readwrite("intensity", &Class::intensity);

    // pcl::Axis
    // pcl::BRISKSignature512
    // pcl::Boundary
    // pcl::CPPFSignature
    // pcl::ESFSignature640
    // pcl::FPFHSignature33
    // pcl::IntensityGradient
    // pcl::InterestPoint
    // pcl::Label
    // pcl::MomentInvariants
    // pcl::Narf36
    // pcl::Normal
    // pcl::NormalBasedSignature12
    // pcl::PFHRGBSignature250
    // pcl::PFHSignature125
    // pcl::PPFRGBSignature
    // pcl::PPFSignature
    // pcl::PointDEM
    // pcl::PointNormal
    // pcl::PointSurfel
    // pcl::PointUV
    // pcl::PointWithRange
    // pcl::PointWithScale
    // pcl::PointWithViewpoint
    // pcl::PointXY
    // pcl::PointXYZ
    // pcl::PointXYZHSV
    // pcl::PointXYZI
    // pcl::PointXYZINormal
    // pcl::PointXYZL
    // pcl::PointXYZLNormal
    // pcl::PointXYZRGB
    // pcl::PointXYZRGBA
    // pcl::PointXYZRGBL
    // pcl::PointXYZRGBNormal
    // pcl::PrincipalCurvatures
    // pcl::PrincipalRadiiRSD
    // pcl::ReferenceFrame
    // pcl::SHOT1344
    // pcl::SHOT352
    // pcl::ShapeContext1980
    // pcl::UniqueShapeContext1960
    // pcl::VFHSignature308
}
