
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/extract_labeled_clusters.h>



template <typename PointT>
void defineSegmentationLabeledEuclideanClusterExtraction(py::module &m, std::string const & suffix) {
    using Class = LabeledEuclideanClusterExtraction<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def_property("cluster_tolerance", &Class::getClusterTolerance, &Class::setClusterTolerance);
    cls.def_property("min_cluster_size", &Class::getMinClusterSize, &Class::setMinClusterSize);
    cls.def_property("max_cluster_size", &Class::getMaxClusterSize, &Class::setMaxClusterSize);
    cls.def_property("max_labels", &Class::getMaxLabels, &Class::setMaxLabels);
    cls.def("extract", &Class::extract);
        
}

void defineSegmentationExtractLabeledClustersClasses(py::module &sub_module) {
    py::module sub_module_LabeledEuclideanClusterExtraction = sub_module.def_submodule("LabeledEuclideanClusterExtraction", "Submodule LabeledEuclideanClusterExtraction");
    defineSegmentationLabeledEuclideanClusterExtraction<PointXYZL>(sub_module_LabeledEuclideanClusterExtraction, "PointXYZL");
    defineSegmentationLabeledEuclideanClusterExtraction<PointXYZLNormal>(sub_module_LabeledEuclideanClusterExtraction, "PointXYZLNormal");
    defineSegmentationLabeledEuclideanClusterExtraction<PointXYZRGBL>(sub_module_LabeledEuclideanClusterExtraction, "PointXYZRGBL");
}