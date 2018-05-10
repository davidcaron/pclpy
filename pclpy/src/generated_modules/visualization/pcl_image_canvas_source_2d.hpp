
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