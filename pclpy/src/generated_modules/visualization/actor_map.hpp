
#include <pcl/visualization/common/actor_map.h>

using namespace pcl::visualization;


void defineVisualizationCloudActor(py::module &m) {
    using Class = pcl::visualization::CloudActor;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "CloudActor");
    cls.def(py::init<>());
    cls.def_readwrite("actor", &Class::actor);
    cls.def_readwrite("geometry_handlers", &Class::geometry_handlers);
    cls.def_readwrite("color_handlers", &Class::color_handlers);
    cls.def_readwrite("color_handler_index_", &Class::color_handler_index_);
    cls.def_readwrite("geometry_handler_index_", &Class::geometry_handler_index_);
    cls.def_readwrite("viewpoint_transformation_", &Class::viewpoint_transformation_);
    cls.def_readwrite("cells", &Class::cells);
}

void defineVisualizationActorMapFunctions(py::module &m) {
}

void defineVisualizationActorMapClasses(py::module &sub_module) {
    defineVisualizationCloudActor(sub_module);
}