
#include <pcl/Vertices.h>



void defineVertices(py::module &m) {
    using Class = pcl::Vertices;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Vertices");
    cls.def(py::init<>());
    cls.def_readwrite("vertices", &Class::vertices);
}

void defineVerticesFunctions(py::module &m) {
}

void defineVerticesClasses(py::module &sub_module) {
    defineVertices(sub_module);
    defineVerticesFunctions(sub_module);
}