
#include <pcl/recognition/point_types.h>



void defineRecognitionGradientXY(py::module &m) {
    using Class = pcl::GradientXY;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "GradientXY");
    cls.def_readonly("data", &Class::data);
    // Operators not implemented (operator<);
}

void defineRecognitionPointTypesFunctions(py::module &m) {
}

void defineRecognitionPointTypesClasses(py::module &sub_module) {
    defineRecognitionGradientXY(sub_module);
    defineRecognitionPointTypesFunctions(sub_module);
}