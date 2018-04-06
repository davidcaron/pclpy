
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/planar_polygon_fusion.h>



template <typename PointT>
void defineSegmentationPlanarPolygonFusion(py::module &m, std::string const & suffix) {
    using Class = PlanarPolygonFusion<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("reset", &Class::reset);
    cls.def("add_input_polygons", &Class::addInputPolygons);
        
}

void defineSegmentationPlanarPolygonFusionClasses(py::module &sub_module) {
    py::module sub_module_PlanarPolygonFusion = sub_module.def_submodule("PlanarPolygonFusion", "Submodule PlanarPolygonFusion");
    defineSegmentationPlanarPolygonFusion<PointXYZ>(sub_module_PlanarPolygonFusion, "PointXYZ");
    defineSegmentationPlanarPolygonFusion<PointXYZI>(sub_module_PlanarPolygonFusion, "PointXYZI");
    defineSegmentationPlanarPolygonFusion<PointXYZRGB>(sub_module_PlanarPolygonFusion, "PointXYZRGB");
    defineSegmentationPlanarPolygonFusion<PointXYZRGBA>(sub_module_PlanarPolygonFusion, "PointXYZRGBA");
}