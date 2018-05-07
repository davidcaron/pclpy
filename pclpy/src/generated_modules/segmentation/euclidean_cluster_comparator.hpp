
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/euclidean_cluster_comparator.h>



template<typename PointT, typename PointNT, typename PointLT>
void defineSegmentationEuclideanClusterComparator(py::module &m, std::string const & suffix) {
    using Class = pcl::EuclideanClusterComparator<PointT, PointNT, PointLT>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using PointCloudL = Class::PointCloudL;
    using PointCloudLPtr = Class::PointCloudLPtr;
    using PointCloudLConstPtr = Class::PointCloudLConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Comparator<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compare", &Class::compare, "idx1"_a, "idx2"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setInputNormals", &Class::setInputNormals, "normals"_a);
    cls.def("setAngularThreshold", &Class::setAngularThreshold, "angular_threshold"_a);
    cls.def("setDistanceThreshold", &Class::setDistanceThreshold, "distance_threshold"_a, "depth_dependent"_a);
    cls.def("setLabels", &Class::setLabels, "labels"_a);
    cls.def("setExcludeLabels", &Class::setExcludeLabels, "exclude_labels"_a);
    cls.def("getInputNormals", &Class::getInputNormals);
    cls.def("getAngularThreshold", &Class::getAngularThreshold);
    cls.def("getDistanceThreshold", &Class::getDistanceThreshold);
        
}

void defineSegmentationEuclideanClusterComparatorFunctions(py::module &m) {
}

void defineSegmentationEuclideanClusterComparatorClasses(py::module &sub_module) {
}