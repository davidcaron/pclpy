
#include <pcl/visualization/window.h>

using namespace pcl::visualization;


void defineVisualizationWindow(py::module &m) {
    using Class = pcl::visualization::Window;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Window");
    cls.def(py::init<std::string>(), "window_name"_a="");
    // Operators not implemented (operator=);
    cls.def("spin", &Class::spin);
    cls.def("spinOnce", &Class::spinOnce, "time"_a=1, "force_redraw"_a=false);
    cls.def("wasStopped", &Class::wasStopped);
}

void defineVisualizationWindowFunctions(py::module &m) {
}

void defineVisualizationWindowClasses(py::module &sub_module) {
    defineVisualizationWindow(sub_module);
}