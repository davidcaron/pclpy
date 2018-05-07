
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/edge_aware_plane_comparator.h>



template<typename PointT, typename PointNT>
void defineSegmentationEdgeAwarePlaneComparator(py::module &m, std::string const & suffix) {
    using Class = pcl::EdgeAwarePlaneComparator<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::PlaneCoefficientComparator<PointT, PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<float*>(), "distance_map"_a);
    cls.def("setDistanceMap", &Class::setDistanceMap, "distance_map"_a);
    cls.def("setCurvatureThreshold", &Class::setCurvatureThreshold, "curvature_threshold"_a);
    cls.def("setDistanceMapThreshold", &Class::setDistanceMapThreshold, "distance_map_threshold"_a);
    cls.def("setEuclideanDistanceThreshold", &Class::setEuclideanDistanceThreshold, "euclidean_distance_threshold"_a);
    cls.def("getDistanceMap", &Class::getDistanceMap);
    cls.def("getCurvatureThreshold", &Class::getCurvatureThreshold);
    cls.def("getDistanceMapThreshold", &Class::getDistanceMapThreshold);
    cls.def("getEuclideanDistanceThreshold", &Class::getEuclideanDistanceThreshold);
        
}

void defineSegmentationEdgeAwarePlaneComparatorFunctions(py::module &m) {
}

void defineSegmentationEdgeAwarePlaneComparatorClasses(py::module &sub_module) {
}