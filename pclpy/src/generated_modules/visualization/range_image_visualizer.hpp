
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/visualization/range_image_visualizer.h>

using namespace pcl::visualization;


void defineVisualizationRangeImageVisualizer(py::module &m) {
    using Class = pcl::visualization::RangeImageVisualizer;
    py::class_<Class, pcl::visualization::ImageViewer, boost::shared_ptr<Class>> cls(m, "RangeImageVisualizer");
    cls.def(py::init<std::string>(), "name"_a="RangeImage");
    cls.def("visualizeBorders", &Class::visualizeBorders, "range_image"_a, "min_value"_a, "max_value"_a, "grayscale"_a, "border_descriptions"_a);
    cls.def("showRangeImage", &Class::showRangeImage, "range_image"_a, "min_value"_a=-std::numeric_limits<float>::infinity(), "max_value"_a=std::numeric_limits<float>::infinity(), "grayscale"_a=false);
    cls.def_static("getRangeImageWidget", &Class::getRangeImageWidget, "range_image"_a, "min_value"_a, "max_value"_a, "grayscale"_a, "name"_a="Rangeimage");
    cls.def_static("getRangeImageBordersWidget", &Class::getRangeImageBordersWidget, "range_image"_a, "min_value"_a, "max_value"_a, "grayscale"_a, "border_descriptions"_a, "name"_a="Rangeimagewithborders");
    cls.def_static("getAnglesWidget", &Class::getAnglesWidget, "range_image"_a, "angles_image"_a, "name"_a);
    cls.def_static("getHalfAnglesWidget", &Class::getHalfAnglesWidget, "range_image"_a, "angles_image"_a, "name"_a);
    cls.def_static("getInterestPointsWidget", &Class::getInterestPointsWidget, "range_image"_a, "interest_image"_a, "min_value"_a, "max_value"_a, "interest_points"_a, "name"_a);
}

void defineVisualizationRangeImageVisualizerFunctions(py::module &m) {
}

void defineVisualizationRangeImageVisualizerClasses(py::module &sub_module) {
    defineVisualizationRangeImageVisualizer(sub_module);
}