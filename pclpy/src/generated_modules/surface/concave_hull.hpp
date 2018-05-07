
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

#pragma warning(disable : 4996)
#include <pcl/surface/concave_hull.h>



template<typename PointInT>
void defineSurfaceConcaveHull(py::module &m, std::string const & suffix) {
    using Class = pcl::ConcaveHull<PointInT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    py::class_<Class, pcl::MeshConstruction<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("reconstruct", py::overload_cast<PointCloud &, std::vector<pcl::Vertices> &> (&Class::reconstruct), "points"_a, "polygons"_a);
    cls.def("reconstruct", py::overload_cast<PointCloud &> (&Class::reconstruct), "output"_a);
    cls.def("setAlpha", &Class::setAlpha, "alpha"_a);
    cls.def("setVoronoiCenters", &Class::setVoronoiCenters, "voronoi_centers"_a);
    cls.def("setKeepInformation", &Class::setKeepInformation, "value"_a);
    cls.def("setDimension", &Class::setDimension, "dimension"_a);
    cls.def("getAlpha", &Class::getAlpha);
    cls.def("getDim", &Class::getDim);
    cls.def("getDimension", &Class::getDimension);
    cls.def("getHullPointIndices", &Class::getHullPointIndices, "hull_point_indices"_a);
        
}

void defineSurfaceConcaveHullFunctions(py::module &m) {
}

void defineSurfaceConcaveHullClasses(py::module &sub_module) {
    py::module sub_module_ConcaveHull = sub_module.def_submodule("ConcaveHull", "Submodule ConcaveHull");
    defineSurfaceConcaveHull<pcl::InterestPoint>(sub_module_ConcaveHull, "InterestPoint");
    defineSurfaceConcaveHull<pcl::PointDEM>(sub_module_ConcaveHull, "PointDEM");
    defineSurfaceConcaveHull<pcl::PointNormal>(sub_module_ConcaveHull, "PointNormal");
    defineSurfaceConcaveHull<pcl::PointSurfel>(sub_module_ConcaveHull, "PointSurfel");
    defineSurfaceConcaveHull<pcl::PointWithRange>(sub_module_ConcaveHull, "PointWithRange");
    defineSurfaceConcaveHull<pcl::PointWithScale>(sub_module_ConcaveHull, "PointWithScale");
    defineSurfaceConcaveHull<pcl::PointWithViewpoint>(sub_module_ConcaveHull, "PointWithViewpoint");
    defineSurfaceConcaveHull<pcl::PointXYZ>(sub_module_ConcaveHull, "PointXYZ");
    defineSurfaceConcaveHull<pcl::PointXYZHSV>(sub_module_ConcaveHull, "PointXYZHSV");
    defineSurfaceConcaveHull<pcl::PointXYZI>(sub_module_ConcaveHull, "PointXYZI");
    defineSurfaceConcaveHull<pcl::PointXYZINormal>(sub_module_ConcaveHull, "PointXYZINormal");
    defineSurfaceConcaveHull<pcl::PointXYZL>(sub_module_ConcaveHull, "PointXYZL");
    defineSurfaceConcaveHull<pcl::PointXYZLNormal>(sub_module_ConcaveHull, "PointXYZLNormal");
    defineSurfaceConcaveHull<pcl::PointXYZRGB>(sub_module_ConcaveHull, "PointXYZRGB");
    defineSurfaceConcaveHull<pcl::PointXYZRGBA>(sub_module_ConcaveHull, "PointXYZRGBA");
    defineSurfaceConcaveHull<pcl::PointXYZRGBL>(sub_module_ConcaveHull, "PointXYZRGBL");
    defineSurfaceConcaveHull<pcl::PointXYZRGBNormal>(sub_module_ConcaveHull, "PointXYZRGBNormal");
}