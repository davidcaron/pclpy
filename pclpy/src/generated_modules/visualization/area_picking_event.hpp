
#include <pcl/visualization/area_picking_event.h>

using namespace pcl::visualization;


void defineVisualizationAreaPickingEvent(py::module &m) {
    using Class = pcl::visualization::AreaPickingEvent;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "AreaPickingEvent");
    cls.def(py::init<int, std::vector<int>>(), "nb_points"_a, "indices"_a);
    cls.def("getPointsIndices", &Class::getPointsIndices, "indices"_a);
}

void defineVisualizationAreaPickingEventFunctions(py::module &m) {
}

void defineVisualizationAreaPickingEventClasses(py::module &sub_module) {
    defineVisualizationAreaPickingEvent(sub_module);
}