
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/edge_aware_plane_comparator.h>



template<typename PointT, typename PointNT>
void defineSegmentationEdgeAwarePlaneComparator(py::module &m, std::string const & suffix) {
    using Class = EdgeAwarePlaneComparator<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PlaneCoefficientComparator<PointT,PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<float*>(), "distance_map"_a);
    cls.def_property("distance_map", &Class::getDistanceMap, &Class::setDistanceMap);
    cls.def_property("curvature_threshold", &Class::getCurvatureThreshold, &Class::setCurvatureThreshold);
    cls.def_property("distance_map_threshold", &Class::getDistanceMapThreshold, &Class::setDistanceMapThreshold);
    cls.def_property("euclidean_distance_threshold", &Class::getEuclideanDistanceThreshold, &Class::setEuclideanDistanceThreshold);
        
}

void defineSegmentationEdgeAwarePlaneComparatorClasses(py::module &sub_module) {
}