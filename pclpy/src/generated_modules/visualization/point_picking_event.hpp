
#include <pcl/visualization/point_picking_event.h>

using namespace pcl::visualization;


void defineVisualizationPointPickingCallback(py::module &m) {
    using Class = pcl::visualization::PointPickingCallback;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PointPickingCallback");
    cls.def_static("New", &Class::New);
    cls.def("Execute", &Class::Execute, "caller"_a, "eventid"_a, ""_a);
    cls.def("performSinglePick", py::overload_cast<vtkRenderWindowInteractor *> (&Class::performSinglePick), "iren"_a);
    cls.def("performSinglePick", py::overload_cast<vtkRenderWindowInteractor *, float &, float &, float &> (&Class::performSinglePick), "iren"_a, "x"_a, "y"_a, "z"_a);
    cls.def("performAreaPick", &Class::performAreaPick, "iren"_a, "indices"_a);
}

void defineVisualizationPointPickingEvent(py::module &m) {
    using Class = pcl::visualization::PointPickingEvent;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PointPickingEvent");
    cls.def(py::init<int>(), "idx"_a);
    cls.def(py::init<int, float, float, float>(), "idx"_a, "x"_a, "y"_a, "z"_a);
    cls.def(py::init<int, int, float, float, float, float, float, float>(), "idx1"_a, "idx2"_a, "x1"_a, "y1"_a, "z1"_a, "x2"_a, "y2"_a, "z2"_a);
    cls.def("getPointIndex", &Class::getPointIndex);
    cls.def("getPoint", &Class::getPoint, "x"_a, "y"_a, "z"_a);
    cls.def("getPoints", &Class::getPoints, "x1"_a, "y1"_a, "z1"_a, "x2"_a, "y2"_a, "z2"_a);
    cls.def("getPointIndices", &Class::getPointIndices, "index_1"_a, "index_2"_a);
}

void defineVisualizationPointPickingEventFunctions(py::module &m) {
}

void defineVisualizationPointPickingEventClasses(py::module &sub_module) {
    defineVisualizationPointPickingCallback(sub_module);
    defineVisualizationPointPickingEvent(sub_module);
}