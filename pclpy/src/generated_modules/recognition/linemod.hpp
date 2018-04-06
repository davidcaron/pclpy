
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/linemod.h>



void defineRecognitionEnergyMaps(py::module &m) {
    using Class = EnergyMaps;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "EnergyMaps");
    cls.def(py::init<>());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("initialize", &Class::initialize);
    cls.def("release_all", &Class::releaseAll);
}

void defineRecognitionLINEMOD(py::module &m) {
    using Class = LINEMOD;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LINEMOD");
    cls.def(py::init<>());
    cls.def("set_detection_threshold", &Class::setDetectionThreshold);
    cls.def("set_non_max_suppression", &Class::setNonMaxSuppression);
    cls.def("set_detection_averaging", &Class::setDetectionAveraging);
    cls.def("load_templates", py::overload_cast<const char *> (&Class::loadTemplates));
    cls.def("load_templates", py::overload_cast<std::vector<std::string> &> (&Class::loadTemplates));
    cls.def("create_and_add_template", &Class::createAndAddTemplate);
    cls.def("add_template", &Class::addTemplate);
    cls.def("detect_templates", &Class::detectTemplates);
    cls.def("detect_templates_semi_scale_invariant", &Class::detectTemplatesSemiScaleInvariant);
    cls.def("match_templates", &Class::matchTemplates);
    cls.def("save_templates", &Class::saveTemplates);
    cls.def("serialize", &Class::serialize);
    cls.def("deserialize", &Class::deserialize);
}

void defineRecognitionLINEMODDetection(py::module &m) {
    using Class = LINEMODDetection;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LINEMODDetection");
    cls.def(py::init<>());
    cls.def_readonly("x", &Class::x);
    cls.def_readonly("y", &Class::y);
    cls.def_readonly("template_id", &Class::template_id);
    cls.def_readonly("score", &Class::score);
    cls.def_readonly("scale", &Class::scale);
}

void defineRecognitionLinearizedMaps(py::module &m) {
    using Class = LinearizedMaps;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LinearizedMaps");
    cls.def(py::init<>());
    // Operators not implemented (operator());
    cls.def("initialize", &Class::initialize);
    cls.def("release_all", &Class::releaseAll);
}

void defineRecognitionLinemodClasses(py::module &sub_module) {
    defineRecognitionEnergyMaps(sub_module);
    defineRecognitionLINEMOD(sub_module);
    defineRecognitionLINEMODDetection(sub_module);
    defineRecognitionLinearizedMaps(sub_module);
}