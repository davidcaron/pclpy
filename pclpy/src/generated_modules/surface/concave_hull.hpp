
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/concave_hull.h>



template<typename PointInT>
void defineSurfaceConcaveHull(py::module &m, std::string const & suffix) {
    using Class = ConcaveHull<PointInT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    py::class_<Class, MeshConstruction<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("alpha", &Class::getAlpha, &Class::setAlpha);
    cls.def("set_voronoi_centers", &Class::setVoronoiCenters);
    cls.def("set_keep_information", &Class::setKeepInformation);
    cls.def_property("dimension", &Class::getDimension, &Class::setDimension);
    cls.def("reconstruct", py::overload_cast<PointCloud &, std::vector<pcl::Vertices> &> (&Class::reconstruct));
    cls.def("reconstruct", py::overload_cast<PointCloud &> (&Class::reconstruct));
        
}

void defineSurfaceConcaveHullClasses(py::module &sub_module) {
    py::module sub_module_ConcaveHull = sub_module.def_submodule("ConcaveHull", "Submodule ConcaveHull");
    defineSurfaceConcaveHull<InterestPoint>(sub_module_ConcaveHull, "InterestPoint");
    defineSurfaceConcaveHull<PointDEM>(sub_module_ConcaveHull, "PointDEM");
    defineSurfaceConcaveHull<PointNormal>(sub_module_ConcaveHull, "PointNormal");
    defineSurfaceConcaveHull<PointSurfel>(sub_module_ConcaveHull, "PointSurfel");
    defineSurfaceConcaveHull<PointWithRange>(sub_module_ConcaveHull, "PointWithRange");
    defineSurfaceConcaveHull<PointWithScale>(sub_module_ConcaveHull, "PointWithScale");
    defineSurfaceConcaveHull<PointWithViewpoint>(sub_module_ConcaveHull, "PointWithViewpoint");
    defineSurfaceConcaveHull<PointXYZ>(sub_module_ConcaveHull, "PointXYZ");
    defineSurfaceConcaveHull<PointXYZHSV>(sub_module_ConcaveHull, "PointXYZHSV");
    defineSurfaceConcaveHull<PointXYZI>(sub_module_ConcaveHull, "PointXYZI");
    defineSurfaceConcaveHull<PointXYZINormal>(sub_module_ConcaveHull, "PointXYZINormal");
    defineSurfaceConcaveHull<PointXYZL>(sub_module_ConcaveHull, "PointXYZL");
    defineSurfaceConcaveHull<PointXYZLNormal>(sub_module_ConcaveHull, "PointXYZLNormal");
    defineSurfaceConcaveHull<PointXYZRGB>(sub_module_ConcaveHull, "PointXYZRGB");
    defineSurfaceConcaveHull<PointXYZRGBA>(sub_module_ConcaveHull, "PointXYZRGBA");
    defineSurfaceConcaveHull<PointXYZRGBL>(sub_module_ConcaveHull, "PointXYZRGBL");
    defineSurfaceConcaveHull<PointXYZRGBNormal>(sub_module_ConcaveHull, "PointXYZRGBNormal");
}