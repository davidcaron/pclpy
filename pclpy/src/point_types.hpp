
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

void definePointTypes(py::module &m) {
    py::module m_pts = m.def_submodule("point_types", "Submodule for point types");

    py::class_<pcl::RGB, boost::shared_ptr<pcl::RGB>> (m_pts, "RGB")
        .def(py::init<>())
        .def_readwrite("rgba", &pcl::RGB::rgba);

    py::class_<pcl::Intensity, boost::shared_ptr<pcl::Intensity>> (m_pts, "Intensity")
        .def(py::init<>())
        .def_readwrite("intensity", &pcl::Intensity::intensity);

    py::class_<pcl::Intensity8u, boost::shared_ptr<pcl::Intensity8u>> (m_pts, "Intensity8u")
        .def(py::init<>())
        .def_readwrite("intensity", &pcl::Intensity8u::intensity);

    py::class_<pcl::Intensity32u, boost::shared_ptr<pcl::Intensity32u>> (m_pts, "Intensity32u")
        .def(py::init<>())
        .def_readwrite("intensity", &pcl::Intensity32u::intensity);

    py::class_<pcl::PointXYZ, boost::shared_ptr<pcl::PointXYZ>> (m_pts, "PointXYZ")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZ::x)
        .def_readwrite("y", &pcl::PointXYZ::y)
        .def_readwrite("z", &pcl::PointXYZ::z);

    py::class_<pcl::PointXYZRGBA, boost::shared_ptr<pcl::PointXYZRGBA>> (m_pts, "PointXYZRGBA")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZRGBA::x)
        .def_readwrite("y", &pcl::PointXYZRGBA::y)
        .def_readwrite("z", &pcl::PointXYZRGBA::z)
        .def_readwrite("rgba", &pcl::PointXYZRGBA::rgba);

    py::class_<pcl::PointXYZRGB, boost::shared_ptr<pcl::PointXYZRGB>> (m_pts, "PointXYZRGB")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZRGB::x)
        .def_readwrite("y", &pcl::PointXYZRGB::y)
        .def_readwrite("z", &pcl::PointXYZRGB::z)
        .def_readwrite("rgb", &pcl::PointXYZRGB::rgb);

    py::class_<pcl::PointXYZRGBL, boost::shared_ptr<pcl::PointXYZRGBL>> (m_pts, "PointXYZRGBL")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZRGBL::x)
        .def_readwrite("y", &pcl::PointXYZRGBL::y)
        .def_readwrite("z", &pcl::PointXYZRGBL::z)
        .def_readwrite("rgba", &pcl::PointXYZRGBL::rgba)
        .def_readwrite("label", &pcl::PointXYZRGBL::label);

    py::class_<pcl::PointXYZHSV, boost::shared_ptr<pcl::PointXYZHSV>> (m_pts, "PointXYZHSV")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZHSV::x)
        .def_readwrite("y", &pcl::PointXYZHSV::y)
        .def_readwrite("z", &pcl::PointXYZHSV::z)
        .def_readwrite("h", &pcl::PointXYZHSV::h)
        .def_readwrite("s", &pcl::PointXYZHSV::s)
        .def_readwrite("v", &pcl::PointXYZHSV::v);

    py::class_<pcl::PointXY, boost::shared_ptr<pcl::PointXY>> (m_pts, "PointXY")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXY::x)
        .def_readwrite("y", &pcl::PointXY::y);

    py::class_<pcl::PointUV, boost::shared_ptr<pcl::PointUV>> (m_pts, "PointUV")
        .def(py::init<>())
        .def_readwrite("u", &pcl::PointUV::u)
        .def_readwrite("v", &pcl::PointUV::v);

    py::class_<pcl::InterestPoint, boost::shared_ptr<pcl::InterestPoint>> (m_pts, "InterestPoint")
        .def(py::init<>())
        .def_readwrite("x", &pcl::InterestPoint::x)
        .def_readwrite("y", &pcl::InterestPoint::y)
        .def_readwrite("z", &pcl::InterestPoint::z)
        .def_readwrite("strength", &pcl::InterestPoint::strength);

    py::class_<pcl::PointXYZI, boost::shared_ptr<pcl::PointXYZI>> (m_pts, "PointXYZI")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZI::x)
        .def_readwrite("y", &pcl::PointXYZI::y)
        .def_readwrite("z", &pcl::PointXYZI::z)
        .def_readwrite("intensity", &pcl::PointXYZI::intensity);

    py::class_<pcl::PointXYZL, boost::shared_ptr<pcl::PointXYZL>> (m_pts, "PointXYZL")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZL::x)
        .def_readwrite("y", &pcl::PointXYZL::y)
        .def_readwrite("z", &pcl::PointXYZL::z)
        .def_readwrite("label", &pcl::PointXYZL::label);

    py::class_<pcl::Label, boost::shared_ptr<pcl::Label>> (m_pts, "Label")
        .def(py::init<>())
        .def_readwrite("label", &pcl::Label::label);

    py::class_<pcl::Normal, boost::shared_ptr<pcl::Normal>> (m_pts, "Normal")
        .def(py::init<>())
        .def_readwrite("normal_x", &pcl::Normal::normal_x)
        .def_readwrite("normal_y", &pcl::Normal::normal_y)
        .def_readwrite("normal_z", &pcl::Normal::normal_z)
        .def_readwrite("curvature", &pcl::Normal::curvature);

    py::class_<pcl::Axis, boost::shared_ptr<pcl::Axis>> (m_pts, "Axis")
        .def(py::init<>())
        .def_readwrite("normal_x", &pcl::Axis::normal_x)
        .def_readwrite("normal_y", &pcl::Axis::normal_y)
        .def_readwrite("normal_z", &pcl::Axis::normal_z);

    py::class_<pcl::PointNormal, boost::shared_ptr<pcl::PointNormal>> (m_pts, "PointNormal")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointNormal::x)
        .def_readwrite("y", &pcl::PointNormal::y)
        .def_readwrite("z", &pcl::PointNormal::z)
        .def_readwrite("normal_x", &pcl::PointNormal::normal_x)
        .def_readwrite("normal_y", &pcl::PointNormal::normal_y)
        .def_readwrite("normal_z", &pcl::PointNormal::normal_z)
        .def_readwrite("curvature", &pcl::PointNormal::curvature);

    py::class_<pcl::PointXYZRGBNormal, boost::shared_ptr<pcl::PointXYZRGBNormal>> (m_pts, "PointXYZRGBNormal")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZRGBNormal::x)
        .def_readwrite("y", &pcl::PointXYZRGBNormal::y)
        .def_readwrite("z", &pcl::PointXYZRGBNormal::z)
        .def_readwrite("rgb", &pcl::PointXYZRGBNormal::rgb)
        .def_readwrite("normal_x", &pcl::PointXYZRGBNormal::normal_x)
        .def_readwrite("normal_y", &pcl::PointXYZRGBNormal::normal_y)
        .def_readwrite("normal_z", &pcl::PointXYZRGBNormal::normal_z)
        .def_readwrite("curvature", &pcl::PointXYZRGBNormal::curvature);

    py::class_<pcl::PointXYZINormal, boost::shared_ptr<pcl::PointXYZINormal>> (m_pts, "PointXYZINormal")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZINormal::x)
        .def_readwrite("y", &pcl::PointXYZINormal::y)
        .def_readwrite("z", &pcl::PointXYZINormal::z)
        .def_readwrite("intensity", &pcl::PointXYZINormal::intensity)
        .def_readwrite("normal_x", &pcl::PointXYZINormal::normal_x)
        .def_readwrite("normal_y", &pcl::PointXYZINormal::normal_y)
        .def_readwrite("normal_z", &pcl::PointXYZINormal::normal_z)
        .def_readwrite("curvature", &pcl::PointXYZINormal::curvature);

    py::class_<pcl::PointXYZLNormal, boost::shared_ptr<pcl::PointXYZLNormal>> (m_pts, "PointXYZLNormal")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZLNormal::x)
        .def_readwrite("y", &pcl::PointXYZLNormal::y)
        .def_readwrite("z", &pcl::PointXYZLNormal::z)
        .def_readwrite("label", &pcl::PointXYZLNormal::label)
        .def_readwrite("normal_x", &pcl::PointXYZLNormal::normal_x)
        .def_readwrite("normal_y", &pcl::PointXYZLNormal::normal_y)
        .def_readwrite("normal_z", &pcl::PointXYZLNormal::normal_z)
        .def_readwrite("curvature", &pcl::PointXYZLNormal::curvature);

    py::class_<pcl::PointWithRange, boost::shared_ptr<pcl::PointWithRange>> (m_pts, "PointWithRange")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointWithRange::x)
        .def_readwrite("y", &pcl::PointWithRange::y)
        .def_readwrite("z", &pcl::PointWithRange::z)
        .def_readwrite("range", &pcl::PointWithRange::range);

    py::class_<pcl::PointWithViewpoint, boost::shared_ptr<pcl::PointWithViewpoint>> (m_pts, "PointWithViewpoint")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointWithViewpoint::x)
        .def_readwrite("y", &pcl::PointWithViewpoint::y)
        .def_readwrite("z", &pcl::PointWithViewpoint::z)
        .def_readwrite("vp_x", &pcl::PointWithViewpoint::vp_x)
        .def_readwrite("vp_y", &pcl::PointWithViewpoint::vp_y)
        .def_readwrite("vp_z", &pcl::PointWithViewpoint::vp_z);

    py::class_<pcl::MomentInvariants, boost::shared_ptr<pcl::MomentInvariants>> (m_pts, "MomentInvariants")
        .def(py::init<>())
        .def_readwrite("j1", &pcl::MomentInvariants::j1)
        .def_readwrite("j2", &pcl::MomentInvariants::j2)
        .def_readwrite("j3", &pcl::MomentInvariants::j3);

    py::class_<pcl::PrincipalRadiiRSD, boost::shared_ptr<pcl::PrincipalRadiiRSD>> (m_pts, "PrincipalRadiiRSD")
        .def(py::init<>())
        .def_readwrite("r_min", &pcl::PrincipalRadiiRSD::r_min)
        .def_readwrite("r_max", &pcl::PrincipalRadiiRSD::r_max);

    py::class_<pcl::Boundary, boost::shared_ptr<pcl::Boundary>> (m_pts, "Boundary")
        .def(py::init<>())
        .def_readwrite("boundary_point", &pcl::Boundary::boundary_point);

    py::class_<pcl::PrincipalCurvatures, boost::shared_ptr<pcl::PrincipalCurvatures>> (m_pts, "PrincipalCurvatures")
        .def(py::init<>())
        .def_readwrite("principal_curvature_x", &pcl::PrincipalCurvatures::principal_curvature_x)
        .def_readwrite("principal_curvature_y", &pcl::PrincipalCurvatures::principal_curvature_y)
        .def_readwrite("principal_curvature_z", &pcl::PrincipalCurvatures::principal_curvature_z)
        .def_readwrite("pc1", &pcl::PrincipalCurvatures::pc1)
        .def_readwrite("pc2", &pcl::PrincipalCurvatures::pc2);

    py::class_<pcl::PFHSignature125, boost::shared_ptr<pcl::PFHSignature125>> (m_pts, "PFHSignature125")
        .def(py::init<>())
        .def_readonly("histogram", &pcl::PFHSignature125::histogram)
        .def_static("descriptorSize", &pcl::PFHSignature125::descriptorSize);

    py::class_<pcl::PFHRGBSignature250, boost::shared_ptr<pcl::PFHRGBSignature250>> (m_pts, "PFHRGBSignature250")
        .def(py::init<>())
        .def_readonly("histogram", &pcl::PFHRGBSignature250::histogram)
        .def_static("descriptorSize", &pcl::PFHRGBSignature250::descriptorSize);

    py::class_<pcl::PPFSignature, boost::shared_ptr<pcl::PPFSignature>> (m_pts, "PPFSignature")
        .def(py::init<>())
        .def_readwrite("f1", &pcl::PPFSignature::f1)
        .def_readwrite("f2", &pcl::PPFSignature::f2)
        .def_readwrite("f3", &pcl::PPFSignature::f3)
        .def_readwrite("f4", &pcl::PPFSignature::f4)
        .def_readwrite("alpha_m", &pcl::PPFSignature::alpha_m);

    py::class_<pcl::CPPFSignature, boost::shared_ptr<pcl::CPPFSignature>> (m_pts, "CPPFSignature")
        .def(py::init<>())
        .def_readwrite("f1", &pcl::CPPFSignature::f1)
        .def_readwrite("f2", &pcl::CPPFSignature::f2)
        .def_readwrite("f3", &pcl::CPPFSignature::f3)
        .def_readwrite("f4", &pcl::CPPFSignature::f4)
        .def_readwrite("f5", &pcl::CPPFSignature::f5)
        .def_readwrite("f6", &pcl::CPPFSignature::f6)
        .def_readwrite("f7", &pcl::CPPFSignature::f7)
        .def_readwrite("f8", &pcl::CPPFSignature::f8)
        .def_readwrite("f9", &pcl::CPPFSignature::f9)
        .def_readwrite("f10", &pcl::CPPFSignature::f10)
        .def_readwrite("alpha_m", &pcl::CPPFSignature::alpha_m);

    py::class_<pcl::PPFRGBSignature, boost::shared_ptr<pcl::PPFRGBSignature>> (m_pts, "PPFRGBSignature")
        .def(py::init<>())
        .def_readwrite("f1", &pcl::PPFRGBSignature::f1)
        .def_readwrite("f2", &pcl::PPFRGBSignature::f2)
        .def_readwrite("f3", &pcl::PPFRGBSignature::f3)
        .def_readwrite("f4", &pcl::PPFRGBSignature::f4)
        .def_readwrite("r_ratio", &pcl::PPFRGBSignature::r_ratio)
        .def_readwrite("g_ratio", &pcl::PPFRGBSignature::g_ratio)
        .def_readwrite("b_ratio", &pcl::PPFRGBSignature::b_ratio)
        .def_readwrite("alpha_m", &pcl::PPFRGBSignature::alpha_m);

    py::class_<pcl::NormalBasedSignature12, boost::shared_ptr<pcl::NormalBasedSignature12>> (m_pts, "NormalBasedSignature12")
        .def(py::init<>())
        .def_readonly("values", &pcl::NormalBasedSignature12::values);

    py::class_<pcl::ShapeContext1980, boost::shared_ptr<pcl::ShapeContext1980>> (m_pts, "ShapeContext1980")
        .def(py::init<>())
        .def_readonly("descriptor", &pcl::ShapeContext1980::descriptor)
        .def_readonly("rf", &pcl::ShapeContext1980::rf)
        .def_static("descriptorSize", &pcl::ShapeContext1980::descriptorSize);

    py::class_<pcl::UniqueShapeContext1960, boost::shared_ptr<pcl::UniqueShapeContext1960>> (m_pts, "UniqueShapeContext1960")
        .def(py::init<>())
        .def_readonly("descriptor", &pcl::UniqueShapeContext1960::descriptor)
        .def_readonly("rf", &pcl::UniqueShapeContext1960::rf)
        .def_static("descriptorSize", &pcl::UniqueShapeContext1960::descriptorSize);

    py::class_<pcl::SHOT352, boost::shared_ptr<pcl::SHOT352>> (m_pts, "SHOT352")
        .def(py::init<>())
        .def_readonly("descriptor", &pcl::SHOT352::descriptor)
        .def_readonly("rf", &pcl::SHOT352::rf)
        .def_static("descriptorSize", &pcl::SHOT352::descriptorSize);

    py::class_<pcl::SHOT1344, boost::shared_ptr<pcl::SHOT1344>> (m_pts, "SHOT1344")
        .def(py::init<>())
        .def_readonly("descriptor", &pcl::SHOT1344::descriptor)
        .def_readonly("rf", &pcl::SHOT1344::rf)
        .def_static("descriptorSize", &pcl::SHOT1344::descriptorSize);

    py::class_<pcl::FPFHSignature33, boost::shared_ptr<pcl::FPFHSignature33>> (m_pts, "FPFHSignature33")
        .def(py::init<>())
        .def_readonly("histogram", &pcl::FPFHSignature33::histogram)
        .def_static("descriptorSize", &pcl::FPFHSignature33::descriptorSize);

    py::class_<pcl::BRISKSignature512, boost::shared_ptr<pcl::BRISKSignature512>> (m_pts, "BRISKSignature512")
        .def(py::init<>())
        .def_readwrite("scale", &pcl::BRISKSignature512::scale)
        .def_readwrite("orientation", &pcl::BRISKSignature512::orientation)
        .def_readonly("descriptor", &pcl::BRISKSignature512::descriptor)
        .def_static("descriptorSize", &pcl::BRISKSignature512::descriptorSize);

    py::class_<pcl::VFHSignature308, boost::shared_ptr<pcl::VFHSignature308>> (m_pts, "VFHSignature308")
        .def(py::init<>())
        .def_readonly("histogram", &pcl::VFHSignature308::histogram)
        .def_static("descriptorSize", &pcl::VFHSignature308::descriptorSize);

    py::class_<pcl::GRSDSignature21, boost::shared_ptr<pcl::GRSDSignature21>> (m_pts, "GRSDSignature21")
        .def(py::init<>())
        .def_readonly("histogram", &pcl::GRSDSignature21::histogram)
        .def_static("descriptorSize", &pcl::GRSDSignature21::descriptorSize);

    py::class_<pcl::ESFSignature640, boost::shared_ptr<pcl::ESFSignature640>> (m_pts, "ESFSignature640")
        .def(py::init<>())
        .def_readonly("histogram", &pcl::ESFSignature640::histogram)
        .def_static("descriptorSize", &pcl::ESFSignature640::descriptorSize);

    py::class_<pcl::Narf36, boost::shared_ptr<pcl::Narf36>> (m_pts, "Narf36")
        .def(py::init<>())
        .def_readonly("descriptor", &pcl::Narf36::descriptor)
        .def_static("descriptorSize", &pcl::Narf36::descriptorSize);

    py::class_<pcl::GFPFHSignature16, boost::shared_ptr<pcl::GFPFHSignature16>> (m_pts, "GFPFHSignature16")
        .def(py::init<>())
        .def_readonly("histogram", &pcl::GFPFHSignature16::histogram);

    py::class_<pcl::IntensityGradient, boost::shared_ptr<pcl::IntensityGradient>> (m_pts, "IntensityGradient")
        .def(py::init<>())
        .def_readwrite("gradient_x", &pcl::IntensityGradient::gradient_x)
        .def_readwrite("gradient_y", &pcl::IntensityGradient::gradient_y)
        .def_readwrite("gradient_z", &pcl::IntensityGradient::gradient_z);

    py::class_<pcl::PointWithScale, boost::shared_ptr<pcl::PointWithScale>> (m_pts, "PointWithScale")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointWithScale::x)
        .def_readwrite("y", &pcl::PointWithScale::y)
        .def_readwrite("z", &pcl::PointWithScale::z)
        .def_readwrite("scale", &pcl::PointWithScale::scale);

    py::class_<pcl::PointSurfel, boost::shared_ptr<pcl::PointSurfel>> (m_pts, "PointSurfel")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointSurfel::x)
        .def_readwrite("y", &pcl::PointSurfel::y)
        .def_readwrite("z", &pcl::PointSurfel::z)
        .def_readwrite("normal_x", &pcl::PointSurfel::normal_x)
        .def_readwrite("normal_y", &pcl::PointSurfel::normal_y)
        .def_readwrite("normal_z", &pcl::PointSurfel::normal_z)
        .def_readwrite("rgba", &pcl::PointSurfel::rgba)
        .def_readwrite("radius", &pcl::PointSurfel::radius)
        .def_readwrite("confidence", &pcl::PointSurfel::confidence)
        .def_readwrite("curvature", &pcl::PointSurfel::curvature);

    py::class_<pcl::ReferenceFrame, boost::shared_ptr<pcl::ReferenceFrame>> (m_pts, "ReferenceFrame")
        .def(py::init<>())
        .def_readonly("x_axis", &pcl::ReferenceFrame::x_axis)
        .def_readonly("y_axis", &pcl::ReferenceFrame::y_axis)
        .def_readonly("z_axis", &pcl::ReferenceFrame::z_axis);

    py::class_<pcl::PointDEM, boost::shared_ptr<pcl::PointDEM>> (m_pts, "PointDEM")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointDEM::x)
        .def_readwrite("y", &pcl::PointDEM::y)
        .def_readwrite("z", &pcl::PointDEM::z)
        .def_readwrite("intensity", &pcl::PointDEM::intensity)
        .def_readwrite("intensity_variance", &pcl::PointDEM::intensity_variance)
        .def_readwrite("height_variance", &pcl::PointDEM::height_variance);

}