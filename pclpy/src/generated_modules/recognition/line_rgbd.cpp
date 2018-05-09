
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/recognition/linemod/line_rgbd.h>



void defineRecognitionBoundingBoxXYZ(py::module &m) {
    using Class = pcl::BoundingBoxXYZ;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "BoundingBoxXYZ");
    cls.def(py::init<>());
    cls.def_readwrite("x", &Class::x);
    cls.def_readwrite("y", &Class::y);
    cls.def_readwrite("z", &Class::z);
    cls.def_readwrite("width", &Class::width);
    cls.def_readwrite("height", &Class::height);
    cls.def_readwrite("depth", &Class::depth);
}

template <typename PointXYZT, typename PointRGBT=PointXYZT>
void defineRecognitionLineRGBD(py::module &m, std::string const & suffix) {
    using Class = pcl::LineRGBD<PointXYZT, PointRGBT=PointXYZT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_readwrite("linemod_", &Class::linemod_);
    cls.def_readwrite("color_gradient_mod_", &Class::color_gradient_mod_);
    cls.def_readwrite("surface_normal_mod_", &Class::surface_normal_mod_);
    cls.def_readwrite("cloud_xyz_", &Class::cloud_xyz_);
    cls.def_readwrite("cloud_rgb_", &Class::cloud_rgb_);
    cls.def_readwrite("template_point_clouds_", &Class::template_point_clouds_);
    cls.def_readwrite("bounding_boxes_", &Class::bounding_boxes_);
    cls.def_readwrite("object_ids_", &Class::object_ids_);
    cls.def_readwrite("detections_", &Class::detections_);
    cls.def("loadTemplates", py::overload_cast<const std::string &, size_t> (&Class::loadTemplates), "file_name"_a, "object_id"_a=0);
    cls.def("addTemplate", &Class::addTemplate, "sqmmt"_a, "cloud"_a, "object_id"_a=0);
    cls.def("createAndAddTemplate", &Class::createAndAddTemplate, "cloud"_a, "object_id"_a, "mask_xyz"_a, "mask_rgb"_a, "region"_a);
    cls.def("detect", &Class::detect, "detections"_a);
    cls.def("detectSemiScaleInvariant", &Class::detectSemiScaleInvariant, "detections"_a, "min_scale"_a=0.6944444f, "max_scale"_a=1.44f, "scale_multiplier"_a=1.2f);
    cls.def("computeTransformedTemplatePoints", &Class::computeTransformedTemplatePoints, "detection_id"_a, "cloud"_a);
    cls.def("findObjectPointIndices", &Class::findObjectPointIndices, "detection_id"_a);
    cls.def("setDetectionThreshold", &Class::setDetectionThreshold, "threshold"_a);
    cls.def("setGradientMagnitudeThreshold", &Class::setGradientMagnitudeThreshold, "threshold"_a);
    cls.def("setIntersectionVolumeThreshold", &Class::setIntersectionVolumeThreshold, "threshold"_a=1.0f);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setInputColors", &Class::setInputColors, "cloud"_a);
        
}

void defineRecognitionLineRgbdFunctions(py::module &m) {
}

void defineRecognitionLineRgbdClasses(py::module &sub_module) {
    defineRecognitionBoundingBoxXYZ(sub_module);
}