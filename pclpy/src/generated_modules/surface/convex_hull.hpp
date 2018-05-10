
#include <pcl/surface/convex_hull.h>



template<typename PointInT>
void defineSurfaceConvexHull(py::module &m, std::string const & suffix) {
    using Class = pcl::ConvexHull<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    py::class_<Class, pcl::MeshConstruction<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("reconstruct", py::overload_cast<PointCloud &, std::vector<pcl::Vertices> &> (&Class::reconstruct), "points"_a, "polygons"_a);
    cls.def("reconstruct", py::overload_cast<PointCloud &> (&Class::reconstruct), "points"_a);
    cls.def("setComputeAreaVolume", &Class::setComputeAreaVolume, "value"_a);
    cls.def("setDimension", &Class::setDimension, "dimension"_a);
    cls.def("getTotalArea", &Class::getTotalArea);
    cls.def("getTotalVolume", &Class::getTotalVolume);
    cls.def("getDimension", &Class::getDimension);
    cls.def("getHullPointIndices", &Class::getHullPointIndices, "hull_point_indices"_a);
        
}

void defineSurfaceConvexHullFunctions1(py::module &m) {
    m.def("comparePoints2D", py::overload_cast<const std::pair<int, Eigen::Vector4f> &, const std::pair<int, Eigen::Vector4f> &> (&pcl::comparePoints2D), "p1"_a, "p2"_a);
}

void defineSurfaceConvexHullFunctions(py::module &m) {
    defineSurfaceConvexHullFunctions1(m);
}

void defineSurfaceConvexHullClasses(py::module &sub_module) {
    py::module sub_module_ConvexHull = sub_module.def_submodule("ConvexHull", "Submodule ConvexHull");
    defineSurfaceConvexHull<pcl::InterestPoint>(sub_module_ConvexHull, "InterestPoint");
    defineSurfaceConvexHull<pcl::PointDEM>(sub_module_ConvexHull, "PointDEM");
    defineSurfaceConvexHull<pcl::PointNormal>(sub_module_ConvexHull, "PointNormal");
    defineSurfaceConvexHull<pcl::PointSurfel>(sub_module_ConvexHull, "PointSurfel");
    defineSurfaceConvexHull<pcl::PointWithRange>(sub_module_ConvexHull, "PointWithRange");
    defineSurfaceConvexHull<pcl::PointWithScale>(sub_module_ConvexHull, "PointWithScale");
    defineSurfaceConvexHull<pcl::PointWithViewpoint>(sub_module_ConvexHull, "PointWithViewpoint");
    defineSurfaceConvexHull<pcl::PointXYZ>(sub_module_ConvexHull, "PointXYZ");
    defineSurfaceConvexHull<pcl::PointXYZHSV>(sub_module_ConvexHull, "PointXYZHSV");
    defineSurfaceConvexHull<pcl::PointXYZI>(sub_module_ConvexHull, "PointXYZI");
    defineSurfaceConvexHull<pcl::PointXYZINormal>(sub_module_ConvexHull, "PointXYZINormal");
    defineSurfaceConvexHull<pcl::PointXYZL>(sub_module_ConvexHull, "PointXYZL");
    defineSurfaceConvexHull<pcl::PointXYZLNormal>(sub_module_ConvexHull, "PointXYZLNormal");
    defineSurfaceConvexHull<pcl::PointXYZRGB>(sub_module_ConvexHull, "PointXYZRGB");
    defineSurfaceConvexHull<pcl::PointXYZRGBA>(sub_module_ConvexHull, "PointXYZRGBA");
    defineSurfaceConvexHull<pcl::PointXYZRGBL>(sub_module_ConvexHull, "PointXYZRGBL");
    defineSurfaceConvexHull<pcl::PointXYZRGBNormal>(sub_module_ConvexHull, "PointXYZRGBNormal");
    defineSurfaceConvexHullFunctions(sub_module);
}