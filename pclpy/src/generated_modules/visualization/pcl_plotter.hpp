
#include <pcl/visualization/pcl_plotter.h>

using namespace pcl::visualization;


void defineVisualizationPCLPlotter(py::module &m) {
    using Class = pcl::visualization::PCLPlotter;
    using PolynomialFunction = Class::PolynomialFunction;
    using RationalFunction = Class::RationalFunction;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PCLPlotter");
    cls.def(py::init<char*>(), "name"_a="PCLPlotter");
    cls.def("addHistogramData", &Class::addHistogramData, "data"_a, "nbins"_a=10, "name"_a="Histogram", "color"_a=std::vector<char>());
    cls.def("plot", &Class::plot);
    cls.def("spinOnce", &Class::spinOnce, "spin_time"_a=1);
    cls.def("spin", &Class::spin);
    cls.def("clearPlots", &Class::clearPlots);
    cls.def("startInteractor", &Class::startInteractor);
    cls.def("renderOnce", &Class::renderOnce);
    cls.def("wasStopped", &Class::wasStopped);
    cls.def("close", &Class::close);
    cls.def("setColorScheme", &Class::setColorScheme, "scheme"_a);
    cls.def("setBackgroundColor", py::overload_cast<const double, const double, const double> (&Class::setBackgroundColor), "r"_a, "g"_a, "b"_a);
    cls.def("setBackgroundColor", py::overload_cast<const double[3]> (&Class::setBackgroundColor), "color"_a);
    cls.def("setXRange", &Class::setXRange, "min"_a, "max"_a);
    cls.def("setYRange", &Class::setYRange, "min"_a, "max"_a);
    cls.def("setTitle", &Class::setTitle, "title"_a);
    cls.def("setXTitle", &Class::setXTitle, "title"_a);
    cls.def("setYTitle", &Class::setYTitle, "title"_a);
    cls.def("setShowLegend", &Class::setShowLegend, "flag"_a);
    cls.def("setWindowSize", &Class::setWindowSize, "w"_a, "h"_a);
    cls.def("setWindowPosition", &Class::setWindowPosition, "x"_a, "y"_a);
    cls.def("setWindowName", &Class::setWindowName, "name"_a);
    cls.def("setViewInteractor", &Class::setViewInteractor, "interactor"_a);
    cls.def("getColorScheme", &Class::getColorScheme);
    cls.def("getBackgroundColor", &Class::getBackgroundColor);
    cls.def("getWindowSize", &Class::getWindowSize);
    cls.def("getRenderWindow", &Class::getRenderWindow);
}

void defineVisualizationPclPlotterFunctions(py::module &m) {
}

void defineVisualizationPclPlotterClasses(py::module &sub_module) {
    defineVisualizationPCLPlotter(sub_module);
}