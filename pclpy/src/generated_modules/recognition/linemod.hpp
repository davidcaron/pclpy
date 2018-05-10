
#include <pcl/recognition/linemod.h>



void defineRecognitionEnergyMaps(py::module &m) {
    using Class = pcl::EnergyMaps;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "EnergyMaps");
    cls.def(py::init<>());
    cls.def("initialize", &Class::initialize, "width"_a, "height"_a, "nr_bins"_a);
    cls.def("releaseAll", &Class::releaseAll);
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
    cls.def("getNumOfBins", &Class::getNumOfBins);
}

void defineRecognitionLINEMOD(py::module &m) {
    using Class = pcl::LINEMOD;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LINEMOD");
    cls.def(py::init<>());
    cls.def("createAndAddTemplate", &Class::createAndAddTemplate, "modalities"_a, "masks"_a, "region"_a);
    cls.def("addTemplate", &Class::addTemplate, "linemod_template"_a);
    cls.def("detectTemplates", &Class::detectTemplates, "modalities"_a, "detections"_a);
    cls.def("detectTemplatesSemiScaleInvariant", &Class::detectTemplatesSemiScaleInvariant, "modalities"_a, "detections"_a, "min_scale"_a=0.6944444f, "max_scale"_a=1.44f, "scale_multiplier"_a=1.2f);
    cls.def("matchTemplates", &Class::matchTemplates, "modalities"_a, "matches"_a);
    cls.def("saveTemplates", &Class::saveTemplates, "file_name"_a);
    cls.def("loadTemplates", py::overload_cast<const char *> (&Class::loadTemplates), "file_name"_a);
    cls.def("loadTemplates", py::overload_cast<std::vector<std::string> &> (&Class::loadTemplates), "file_names"_a);
    cls.def("serialize", &Class::serialize, "stream"_a);
    cls.def("deserialize", &Class::deserialize, "stream"_a);
    cls.def("setDetectionThreshold", &Class::setDetectionThreshold, "threshold"_a);
    cls.def("setNonMaxSuppression", &Class::setNonMaxSuppression, "use_non_max_suppression"_a);
    cls.def("setDetectionAveraging", &Class::setDetectionAveraging, "average_detections"_a);
    cls.def("getTemplate", &Class::getTemplate, "template_id"_a);
    cls.def("getNumOfTemplates", &Class::getNumOfTemplates);
}

void defineRecognitionLINEMODDetection(py::module &m) {
    using Class = pcl::LINEMODDetection;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LINEMODDetection");
    cls.def(py::init<>());
    cls.def_readwrite("x", &Class::x);
    cls.def_readwrite("y", &Class::y);
    cls.def_readwrite("template_id", &Class::template_id);
    cls.def_readwrite("score", &Class::score);
    cls.def_readwrite("scale", &Class::scale);
}

void defineRecognitionLinearizedMaps(py::module &m) {
    using Class = pcl::LinearizedMaps;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LinearizedMaps");
    cls.def(py::init<>());
    cls.def("initialize", &Class::initialize, "width"_a, "height"_a, "step_size"_a);
    cls.def("releaseAll", &Class::releaseAll);
    // Operators not implemented (operator());
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
    cls.def("getStepSize", &Class::getStepSize);
    cls.def("getMapMemorySize", &Class::getMapMemorySize);
    cls.def("getOffsetMap", &Class::getOffsetMap, "col_index"_a, "row_index"_a);
}

void defineRecognitionLinemodFunctions(py::module &m) {
}

void defineRecognitionLinemodClasses(py::module &sub_module) {
    defineRecognitionEnergyMaps(sub_module);
    defineRecognitionLINEMOD(sub_module);
    defineRecognitionLINEMODDetection(sub_module);
    defineRecognitionLinearizedMaps(sub_module);
}