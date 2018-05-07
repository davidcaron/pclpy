
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/extract_labeled_clusters.h>



template <typename PointT>
void defineSegmentationLabeledEuclideanClusterExtraction(py::module &m, std::string const & suffix) {
    using Class = pcl::LabeledEuclideanClusterExtraction<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("extract", &Class::extract, "labeled_clusters"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("setClusterTolerance", &Class::setClusterTolerance, "tolerance"_a);
    cls.def("setMinClusterSize", &Class::setMinClusterSize, "min_cluster_size"_a);
    cls.def("setMaxClusterSize", &Class::setMaxClusterSize, "max_cluster_size"_a);
    cls.def("setMaxLabels", &Class::setMaxLabels, "max_label"_a);
    cls.def("getSearchMethod", &Class::getSearchMethod);
    cls.def("getClusterTolerance", &Class::getClusterTolerance);
    cls.def("getMinClusterSize", &Class::getMinClusterSize);
    cls.def("getMaxClusterSize", &Class::getMaxClusterSize);
    cls.def("getMaxLabels", &Class::getMaxLabels);
        
}

void defineSegmentationExtractLabeledClustersFunctions1(py::module &m) {
    m.def("compareLabeledPointClusters", py::overload_cast<const pcl::PointIndices &, const pcl::PointIndices &> (&pcl::compareLabeledPointClusters), "a"_a, "b"_a);
}

void defineSegmentationExtractLabeledClustersFunctions(py::module &m) {
    defineSegmentationExtractLabeledClustersFunctions1(m);
}

void defineSegmentationExtractLabeledClustersClasses(py::module &sub_module) {
    py::module sub_module_LabeledEuclideanClusterExtraction = sub_module.def_submodule("LabeledEuclideanClusterExtraction", "Submodule LabeledEuclideanClusterExtraction");
    defineSegmentationLabeledEuclideanClusterExtraction<pcl::PointXYZL>(sub_module_LabeledEuclideanClusterExtraction, "PointXYZL");
    defineSegmentationLabeledEuclideanClusterExtraction<pcl::PointXYZLNormal>(sub_module_LabeledEuclideanClusterExtraction, "PointXYZLNormal");
    defineSegmentationLabeledEuclideanClusterExtraction<pcl::PointXYZRGBL>(sub_module_LabeledEuclideanClusterExtraction, "PointXYZRGBL");
    defineSegmentationExtractLabeledClustersFunctions(sub_module);
}