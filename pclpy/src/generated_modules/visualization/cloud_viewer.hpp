
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/visualization/cloud_viewer.h>

using namespace pcl::visualization;


void defineVisualizationCloudViewer(py::module &m) {
    using Class = pcl::visualization::CloudViewer;
    using ColorACloud = Class::ColorACloud;
    using ColorCloud = Class::ColorCloud;
    using GrayCloud = Class::GrayCloud;
    using MonochromeCloud = Class::MonochromeCloud;
    using VizCallable = Class::VizCallable;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "CloudViewer");
    cls.def(py::init<std::string>(), "window_name"_a);
    cls.def("showCloud", py::overload_cast<const Class::ColorCloud::ConstPtr &, const std::string &> (&Class::showCloud), "cloud"_a, "cloudname"_a="cloud");
    cls.def("showCloud", py::overload_cast<const Class::ColorACloud::ConstPtr &, const std::string &> (&Class::showCloud), "cloud"_a, "cloudname"_a="cloud");
    cls.def("showCloud", py::overload_cast<const Class::GrayCloud::ConstPtr &, const std::string &> (&Class::showCloud), "cloud"_a, "cloudname"_a="cloud");
    cls.def("showCloud", py::overload_cast<const Class::MonochromeCloud::ConstPtr &, const std::string &> (&Class::showCloud), "cloud"_a, "cloudname"_a="cloud");
    cls.def("wasStopped", &Class::wasStopped, "millis_to_wait"_a=1);
    cls.def("runOnVisualizationThread", &Class::runOnVisualizationThread, "x"_a, "key"_a="callable");
    cls.def("runOnVisualizationThreadOnce", &Class::runOnVisualizationThreadOnce, "x"_a);
    cls.def("removeVisualizationCallable", &Class::removeVisualizationCallable, "key"_a="callable");
}

void defineVisualizationCloudViewerFunctions(py::module &m) {
}

void defineVisualizationCloudViewerClasses(py::module &sub_module) {
    defineVisualizationCloudViewer(sub_module);
}