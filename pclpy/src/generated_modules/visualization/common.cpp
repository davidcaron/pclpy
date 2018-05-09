
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/visualization/common/common.h>

using namespace pcl::visualization;


void defineVisualizationCamera(py::module &m) {
    using Class = pcl::visualization::Camera;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Camera");
    cls.def_readonly("focal", &Class::focal);
    cls.def_readonly("pos", &Class::pos);
    cls.def_readonly("view", &Class::view);
    cls.def_readonly("clip", &Class::clip);
    cls.def_readwrite("fovy", &Class::fovy);
    cls.def_readonly("window_size", &Class::window_size);
    cls.def_readonly("window_pos", &Class::window_pos);
    cls.def("computeViewMatrix", &Class::computeViewMatrix, "view_mat"_a);
    cls.def("computeProjectionMatrix", &Class::computeProjectionMatrix, "proj"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointSurfel &, Eigen::Vector4d &> (&Class::cvtWindowCoordinates<pcl::PointSurfel>, py::const_), "pt"_a, "window_cord"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZ &, Eigen::Vector4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZ>, py::const_), "pt"_a, "window_cord"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZL &, Eigen::Vector4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZL>, py::const_), "pt"_a, "window_cord"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZI &, Eigen::Vector4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZI>, py::const_), "pt"_a, "window_cord"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZRGB &, Eigen::Vector4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZRGB>, py::const_), "pt"_a, "window_cord"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZRGBA &, Eigen::Vector4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZRGBA>, py::const_), "pt"_a, "window_cord"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointNormal &, Eigen::Vector4d &> (&Class::cvtWindowCoordinates<pcl::PointNormal>, py::const_), "pt"_a, "window_cord"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZRGBNormal &, Eigen::Vector4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZRGBNormal>, py::const_), "pt"_a, "window_cord"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZRGBL &, Eigen::Vector4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZRGBL>, py::const_), "pt"_a, "window_cord"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointWithRange &, Eigen::Vector4d &> (&Class::cvtWindowCoordinates<pcl::PointWithRange>, py::const_), "pt"_a, "window_cord"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointSurfel &, Eigen::Vector4d &, const Eigen::Matrix4d &> (&Class::cvtWindowCoordinates<pcl::PointSurfel>, py::const_), "pt"_a, "window_cord"_a, "composite_mat"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZ &, Eigen::Vector4d &, const Eigen::Matrix4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZ>, py::const_), "pt"_a, "window_cord"_a, "composite_mat"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZL &, Eigen::Vector4d &, const Eigen::Matrix4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZL>, py::const_), "pt"_a, "window_cord"_a, "composite_mat"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZI &, Eigen::Vector4d &, const Eigen::Matrix4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZI>, py::const_), "pt"_a, "window_cord"_a, "composite_mat"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZRGB &, Eigen::Vector4d &, const Eigen::Matrix4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZRGB>, py::const_), "pt"_a, "window_cord"_a, "composite_mat"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZRGBA &, Eigen::Vector4d &, const Eigen::Matrix4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZRGBA>, py::const_), "pt"_a, "window_cord"_a, "composite_mat"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointNormal &, Eigen::Vector4d &, const Eigen::Matrix4d &> (&Class::cvtWindowCoordinates<pcl::PointNormal>, py::const_), "pt"_a, "window_cord"_a, "composite_mat"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZRGBNormal &, Eigen::Vector4d &, const Eigen::Matrix4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZRGBNormal>, py::const_), "pt"_a, "window_cord"_a, "composite_mat"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointXYZRGBL &, Eigen::Vector4d &, const Eigen::Matrix4d &> (&Class::cvtWindowCoordinates<pcl::PointXYZRGBL>, py::const_), "pt"_a, "window_cord"_a, "composite_mat"_a);
    cls.def("cvtWindowCoordinates", py::overload_cast<const pcl::PointWithRange &, Eigen::Vector4d &, const Eigen::Matrix4d &> (&Class::cvtWindowCoordinates<pcl::PointWithRange>, py::const_), "pt"_a, "window_cord"_a, "composite_mat"_a);
}

void defineVisualizationCommonFunctions(py::module &m) {
}

void defineVisualizationCommonClasses(py::module &sub_module) {
    defineVisualizationCamera(sub_module);
    py::enum_<pcl::visualization::FrustumCull>(sub_module, "FrustumCull")
        .value("PCL_INSIDE_FRUSTUM", pcl::visualization::FrustumCull::PCL_INSIDE_FRUSTUM)
        .value("PCL_INTERSECT_FRUSTUM", pcl::visualization::FrustumCull::PCL_INTERSECT_FRUSTUM)
        .value("PCL_OUTSIDE_FRUSTUM", pcl::visualization::FrustumCull::PCL_OUTSIDE_FRUSTUM)
        .export_values();
    py::enum_<pcl::visualization::LookUpTableRepresentationProperties>(sub_module, "LookUpTableRepresentationProperties")
        .value("PCL_VISUALIZER_LUT_JET", pcl::visualization::LookUpTableRepresentationProperties::PCL_VISUALIZER_LUT_JET)
        .value("PCL_VISUALIZER_LUT_JET_INVERSE", pcl::visualization::LookUpTableRepresentationProperties::PCL_VISUALIZER_LUT_JET_INVERSE)
        .value("PCL_VISUALIZER_LUT_HSV", pcl::visualization::LookUpTableRepresentationProperties::PCL_VISUALIZER_LUT_HSV)
        .value("PCL_VISUALIZER_LUT_HSV_INVERSE", pcl::visualization::LookUpTableRepresentationProperties::PCL_VISUALIZER_LUT_HSV_INVERSE)
        .value("PCL_VISUALIZER_LUT_GREY", pcl::visualization::LookUpTableRepresentationProperties::PCL_VISUALIZER_LUT_GREY)
        .value("PCL_VISUALIZER_LUT_BLUE2RED", pcl::visualization::LookUpTableRepresentationProperties::PCL_VISUALIZER_LUT_BLUE2RED)
        .value("PCL_VISUALIZER_LUT_RANGE_AUTO", pcl::visualization::LookUpTableRepresentationProperties::PCL_VISUALIZER_LUT_RANGE_AUTO)
        .export_values();
    py::enum_<pcl::visualization::RenderingProperties>(sub_module, "RenderingProperties")
        .value("PCL_VISUALIZER_POINT_SIZE", pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE)
        .value("PCL_VISUALIZER_OPACITY", pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY)
        .value("PCL_VISUALIZER_LINE_WIDTH", pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH)
        .value("PCL_VISUALIZER_FONT_SIZE", pcl::visualization::RenderingProperties::PCL_VISUALIZER_FONT_SIZE)
        .value("PCL_VISUALIZER_COLOR", pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR)
        .value("PCL_VISUALIZER_REPRESENTATION", pcl::visualization::RenderingProperties::PCL_VISUALIZER_REPRESENTATION)
        .value("PCL_VISUALIZER_IMMEDIATE_RENDERING", pcl::visualization::RenderingProperties::PCL_VISUALIZER_IMMEDIATE_RENDERING)
        .value("PCL_VISUALIZER_SHADING", pcl::visualization::RenderingProperties::PCL_VISUALIZER_SHADING)
        .value("PCL_VISUALIZER_LUT", pcl::visualization::RenderingProperties::PCL_VISUALIZER_LUT)
        .value("PCL_VISUALIZER_LUT_RANGE", pcl::visualization::RenderingProperties::PCL_VISUALIZER_LUT_RANGE)
        .export_values();
    py::enum_<pcl::visualization::RenderingRepresentationProperties>(sub_module, "RenderingRepresentationProperties")
        .value("PCL_VISUALIZER_REPRESENTATION_POINTS", pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_POINTS)
        .value("PCL_VISUALIZER_REPRESENTATION_WIREFRAME", pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_WIREFRAME)
        .value("PCL_VISUALIZER_REPRESENTATION_SURFACE", pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_SURFACE)
        .export_values();
    py::enum_<pcl::visualization::ShadingRepresentationProperties>(sub_module, "ShadingRepresentationProperties")
        .value("PCL_VISUALIZER_SHADING_FLAT", pcl::visualization::ShadingRepresentationProperties::PCL_VISUALIZER_SHADING_FLAT)
        .value("PCL_VISUALIZER_SHADING_GOURAUD", pcl::visualization::ShadingRepresentationProperties::PCL_VISUALIZER_SHADING_GOURAUD)
        .value("PCL_VISUALIZER_SHADING_PHONG", pcl::visualization::ShadingRepresentationProperties::PCL_VISUALIZER_SHADING_PHONG)
        .export_values();
}