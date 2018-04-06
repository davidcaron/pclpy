
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/euclidean_cluster_comparator.h>



template<typename PointT, typename PointNT, typename PointLT>
void defineSegmentationEuclideanClusterComparator(py::module &m, std::string const & suffix) {
    using Class = EuclideanClusterComparator<PointT, PointNT, PointLT>;
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
    py::class_<Class, Comparator<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def_property("input_normals", &Class::getInputNormals, &Class::setInputNormals);
    cls.def_property("angular_threshold", &Class::getAngularThreshold, &Class::setAngularThreshold);
    cls.def_property("distance_threshold", &Class::getDistanceThreshold, &Class::setDistanceThreshold);
    cls.def("set_labels", &Class::setLabels);
    cls.def("set_exclude_labels", &Class::setExcludeLabels);
    cls.def("compare", &Class::compare);
        
}

void defineSegmentationEuclideanClusterComparatorClasses(py::module &sub_module) {
}