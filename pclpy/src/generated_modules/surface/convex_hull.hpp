
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/convex_hull.h>



template<typename PointInT>
void defineSurfaceConvexHull(py::module &m, std::string const & suffix) {
    using Class = ConvexHull<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    py::class_<Class, MeshConstruction<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_compute_area_volume", &Class::setComputeAreaVolume);
    cls.def_property("dimension", &Class::getDimension, &Class::setDimension);
    cls.def("reconstruct", py::overload_cast<PointCloud &, std::vector<pcl::Vertices> &> (&Class::reconstruct));
    cls.def("reconstruct", py::overload_cast<PointCloud &> (&Class::reconstruct));
        
}

void defineSurfaceConvexHullClasses(py::module &sub_module) {
    py::module sub_module_ConvexHull = sub_module.def_submodule("ConvexHull", "Submodule ConvexHull");
    defineSurfaceConvexHull<InterestPoint>(sub_module_ConvexHull, "InterestPoint");
    defineSurfaceConvexHull<PointDEM>(sub_module_ConvexHull, "PointDEM");
    defineSurfaceConvexHull<PointNormal>(sub_module_ConvexHull, "PointNormal");
    defineSurfaceConvexHull<PointSurfel>(sub_module_ConvexHull, "PointSurfel");
    defineSurfaceConvexHull<PointWithRange>(sub_module_ConvexHull, "PointWithRange");
    defineSurfaceConvexHull<PointWithScale>(sub_module_ConvexHull, "PointWithScale");
    defineSurfaceConvexHull<PointWithViewpoint>(sub_module_ConvexHull, "PointWithViewpoint");
    defineSurfaceConvexHull<PointXYZ>(sub_module_ConvexHull, "PointXYZ");
    defineSurfaceConvexHull<PointXYZHSV>(sub_module_ConvexHull, "PointXYZHSV");
    defineSurfaceConvexHull<PointXYZI>(sub_module_ConvexHull, "PointXYZI");
    defineSurfaceConvexHull<PointXYZINormal>(sub_module_ConvexHull, "PointXYZINormal");
    defineSurfaceConvexHull<PointXYZL>(sub_module_ConvexHull, "PointXYZL");
    defineSurfaceConvexHull<PointXYZLNormal>(sub_module_ConvexHull, "PointXYZLNormal");
    defineSurfaceConvexHull<PointXYZRGB>(sub_module_ConvexHull, "PointXYZRGB");
    defineSurfaceConvexHull<PointXYZRGBA>(sub_module_ConvexHull, "PointXYZRGBA");
    defineSurfaceConvexHull<PointXYZRGBL>(sub_module_ConvexHull, "PointXYZRGBL");
    defineSurfaceConvexHull<PointXYZRGBNormal>(sub_module_ConvexHull, "PointXYZRGBNormal");
}