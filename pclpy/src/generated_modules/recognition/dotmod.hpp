
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/dotmod.h>



void defineRecognitionDOTMOD(py::module &m) {
    using Class = DOTMOD;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DOTMOD");
    cls.def(py::init<size_t, size_t>(), "template_width"_a, "template_height"_a);
    cls.def("load_templates", py::overload_cast<const char *> (&Class::loadTemplates));
    cls.def("create_and_add_template", &Class::createAndAddTemplate);
    cls.def("detect_templates", &Class::detectTemplates);
    cls.def("save_templates", &Class::saveTemplates);
    cls.def("serialize", &Class::serialize);
    cls.def("deserialize", &Class::deserialize);
}

void defineRecognitionDOTMODDetection(py::module &m) {
    using Class = DOTMODDetection;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DOTMODDetection");
    cls.def_readonly("bin_x", &Class::bin_x);
    cls.def_readonly("bin_y", &Class::bin_y);
    cls.def_readonly("template_id", &Class::template_id);
    cls.def_readonly("score", &Class::score);
}

void defineRecognitionDotmodClasses(py::module &sub_module) {
    defineRecognitionDOTMOD(sub_module);
    defineRecognitionDOTMODDetection(sub_module);
}