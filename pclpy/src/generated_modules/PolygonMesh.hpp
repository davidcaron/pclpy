
#include <pcl/PolygonMesh.h>



void definePolygonMesh(py::module &m) {
    using Class = pcl::PolygonMesh;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PolygonMesh");
    cls.def(py::init<>());
    cls.def_readwrite("header", &Class::header);
    cls.def_readwrite("cloud", &Class::cloud);
    cls.def_readwrite("polygons", &Class::polygons);
}

void definePolygonMeshFunctions(py::module &m) {
}

void definePolygonMeshClasses(py::module &sub_module) {
    definePolygonMesh(sub_module);
    definePolygonMeshFunctions(sub_module);
}