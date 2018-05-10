
#include <pcl/ModelCoefficients.h>



void defineModelCoefficients(py::module &m) {
    using Class = pcl::ModelCoefficients;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "ModelCoefficients");
    cls.def(py::init<>());
    cls.def_readwrite("header", &Class::header);
    cls.def_readwrite("values", &Class::values);
}

void defineModelCoefficientsFunctions(py::module &m) {
}

void defineModelCoefficientsClasses(py::module &sub_module) {
    defineModelCoefficients(sub_module);
    defineModelCoefficientsFunctions(sub_module);
}