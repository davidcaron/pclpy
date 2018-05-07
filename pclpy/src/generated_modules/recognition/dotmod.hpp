
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/recognition/dotmod.h>



void defineRecognitionDOTMOD(py::module &m) {
    using Class = pcl::DOTMOD;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DOTMOD");
    cls.def(py::init<size_t, size_t>(), "template_width"_a, "template_height"_a);
    cls.def("createAndAddTemplate", &Class::createAndAddTemplate, "modalities"_a, "masks"_a, "template_anker_x"_a, "template_anker_y"_a, "region"_a);
    cls.def("detectTemplates", &Class::detectTemplates, "modalities"_a, "template_response_threshold"_a, "detections"_a, "bin_size"_a);
    cls.def("saveTemplates", &Class::saveTemplates, "file_name"_a);
    cls.def("loadTemplates", py::overload_cast<const char *> (&Class::loadTemplates), "file_name"_a);
    cls.def("serialize", &Class::serialize, "stream"_a);
    cls.def("deserialize", &Class::deserialize, "stream"_a);
    cls.def("getTemplate", &Class::getTemplate, "template_id"_a);
    cls.def("getNumOfTemplates", &Class::getNumOfTemplates);
}

void defineRecognitionDOTMODDetection(py::module &m) {
    using Class = pcl::DOTMODDetection;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DOTMODDetection");
    cls.def_readwrite("bin_x", &Class::bin_x);
    cls.def_readwrite("bin_y", &Class::bin_y);
    cls.def_readwrite("template_id", &Class::template_id);
    cls.def_readwrite("score", &Class::score);
}

void defineRecognitionDotmodFunctions(py::module &m) {
}

void defineRecognitionDotmodClasses(py::module &sub_module) {
    defineRecognitionDOTMOD(sub_module);
    defineRecognitionDOTMODDetection(sub_module);
}