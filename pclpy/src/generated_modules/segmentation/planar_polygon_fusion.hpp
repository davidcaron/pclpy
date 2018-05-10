
#include <pcl/segmentation/planar_polygon_fusion.h>



template <typename PointT>
void defineSegmentationPlanarPolygonFusion(py::module &m, std::string const & suffix) {
    using Class = pcl::PlanarPolygonFusion<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("reset", &Class::reset);
    cls.def("addInputPolygons", &Class::addInputPolygons, "input"_a);
        
}

void defineSegmentationPlanarPolygonFusionFunctions(py::module &m) {
}

void defineSegmentationPlanarPolygonFusionClasses(py::module &sub_module) {
    py::module sub_module_PlanarPolygonFusion = sub_module.def_submodule("PlanarPolygonFusion", "Submodule PlanarPolygonFusion");
    defineSegmentationPlanarPolygonFusion<pcl::PointXYZ>(sub_module_PlanarPolygonFusion, "PointXYZ");
    defineSegmentationPlanarPolygonFusion<pcl::PointXYZI>(sub_module_PlanarPolygonFusion, "PointXYZI");
    defineSegmentationPlanarPolygonFusion<pcl::PointXYZRGB>(sub_module_PlanarPolygonFusion, "PointXYZRGB");
    defineSegmentationPlanarPolygonFusion<pcl::PointXYZRGBA>(sub_module_PlanarPolygonFusion, "PointXYZRGBA");
}