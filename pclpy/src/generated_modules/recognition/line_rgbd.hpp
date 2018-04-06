
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/line_rgbd.h>



void defineRecognitionBoundingBoxXYZ(py::module &m) {
    using Class = BoundingBoxXYZ;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "BoundingBoxXYZ");
    cls.def(py::init<>());
    cls.def_readonly("x", &Class::x);
    cls.def_readonly("y", &Class::y);
    cls.def_readonly("z", &Class::z);
    cls.def_readonly("width", &Class::width);
    cls.def_readonly("height", &Class::height);
    cls.def_readonly("depth", &Class::depth);
}

template <typename PointXYZT, typename PointRGBT=PointXYZT>
void defineRecognitionLineRGBD(py::module &m, std::string const & suffix) {
    using Class = LineRGBD<PointXYZT, PointRGBT=PointXYZT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_detection_threshold", &Class::setDetectionThreshold);
    cls.def("set_gradient_magnitude_threshold", &Class::setGradientMagnitudeThreshold);
    cls.def("set_intersection_volume_threshold", &Class::setIntersectionVolumeThreshold);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("set_input_colors", &Class::setInputColors);
    cls.def_readonly("linemod_", &Class::linemod_);
    cls.def_readonly("color_gradient_mod_", &Class::color_gradient_mod_);
    cls.def_readonly("surface_normal_mod_", &Class::surface_normal_mod_);
    cls.def_readonly("cloud_xyz_", &Class::cloud_xyz_);
    cls.def_readonly("cloud_rgb_", &Class::cloud_rgb_);
    cls.def_readonly("template_point_clouds_", &Class::template_point_clouds_);
    cls.def_readonly("bounding_boxes_", &Class::bounding_boxes_);
    cls.def_readonly("object_ids_", &Class::object_ids_);
    cls.def_readonly("detections_", &Class::detections_);
    cls.def("load_templates", py::overload_cast<const std::string &, size_t> (&Class::loadTemplates));
    cls.def("add_template", &Class::addTemplate);
    cls.def("create_and_add_template", &Class::createAndAddTemplate);
    cls.def("detect", &Class::detect);
    cls.def("detect_semi_scale_invariant", &Class::detectSemiScaleInvariant);
    cls.def("compute_transformed_template_points", &Class::computeTransformedTemplatePoints);
    cls.def("find_object_point_indices", &Class::findObjectPointIndices);
        
}

void defineRecognitionLineRgbdClasses(py::module &sub_module) {
    defineRecognitionBoundingBoxXYZ(sub_module);
}