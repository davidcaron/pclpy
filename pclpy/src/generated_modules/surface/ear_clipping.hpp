
#include <pcl/surface/ear_clipping.h>



void defineSurfaceEarClipping(py::module &m) {
    using Class = pcl::EarClipping;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::MeshProcessing, boost::shared_ptr<Class>> cls(m, "EarClipping");
    cls.def(py::init<>());
}

void defineSurfaceEarClippingFunctions(py::module &m) {
}

void defineSurfaceEarClippingClasses(py::module &sub_module) {
    defineSurfaceEarClipping(sub_module);
}