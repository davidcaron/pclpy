
#include <pcl/segmentation/extract_clusters.h>



template <typename PointT>
void defineSegmentationEuclideanClusterExtraction(py::module &m, std::string const & suffix) {
    using Class = pcl::EuclideanClusterExtraction<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("extract", &Class::extract, "clusters"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("setClusterTolerance", &Class::setClusterTolerance, "tolerance"_a);
    cls.def("setMinClusterSize", &Class::setMinClusterSize, "min_cluster_size"_a);
    cls.def("setMaxClusterSize", &Class::setMaxClusterSize, "max_cluster_size"_a);
    cls.def("getSearchMethod", &Class::getSearchMethod);
    cls.def("getClusterTolerance", &Class::getClusterTolerance);
    cls.def("getMinClusterSize", &Class::getMinClusterSize);
    cls.def("getMaxClusterSize", &Class::getMaxClusterSize);
        
}

void defineSegmentationExtractClustersFunctions1(py::module &m) {
    m.def("comparePointClusters", py::overload_cast<const pcl::PointIndices &, const pcl::PointIndices &> (&pcl::comparePointClusters), "a"_a, "b"_a);
}

void defineSegmentationExtractClustersFunctions(py::module &m) {
    defineSegmentationExtractClustersFunctions1(m);
}

void defineSegmentationExtractClustersClasses(py::module &sub_module) {
    py::module sub_module_EuclideanClusterExtraction = sub_module.def_submodule("EuclideanClusterExtraction", "Submodule EuclideanClusterExtraction");
    defineSegmentationEuclideanClusterExtraction<pcl::PointXYZ>(sub_module_EuclideanClusterExtraction, "PointXYZ");
    defineSegmentationEuclideanClusterExtraction<pcl::PointXYZI>(sub_module_EuclideanClusterExtraction, "PointXYZI");
    defineSegmentationEuclideanClusterExtraction<pcl::PointXYZRGB>(sub_module_EuclideanClusterExtraction, "PointXYZRGB");
    defineSegmentationEuclideanClusterExtraction<pcl::PointXYZRGBA>(sub_module_EuclideanClusterExtraction, "PointXYZRGBA");
    defineSegmentationExtractClustersFunctions(sub_module);
}