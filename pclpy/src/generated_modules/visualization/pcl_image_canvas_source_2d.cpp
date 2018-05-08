
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/visualization/vtk/pcl_image_canvas_source_2d.h>

using namespace pcl::visualization;


void defineVisualizationPCLImageCanvasSource2D(py::module &m) {
    using Class = pcl::visualization::PCLImageCanvasSource2D;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PCLImageCanvasSource2D");
    cls.def_static("New", &Class::New);
    cls.def("DrawImage", &Class::DrawImage, "image"_a);
}

void defineVisualizationPclImageCanvasSource2dFunctions(py::module &m) {
}

void defineVisualizationPclImageCanvasSource2dClasses(py::module &sub_module) {
    defineVisualizationPCLImageCanvasSource2D(sub_module);
}