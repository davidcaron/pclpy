
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/supervoxel_clustering.h>



template <typename PointT>
void defineSegmentationSupervoxel(py::module &m, std::string const & suffix) {
    using Class = pcl::Supervoxel<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_readwrite("normal_", &Class::normal_);
    cls.def_readwrite("centroid_", &Class::centroid_);
    cls.def_readwrite("voxels_", &Class::voxels_);
    cls.def_readwrite("normals_", &Class::normals_);
    cls.def("getCentroidPoint", &Class::getCentroidPoint, "centroid_arg"_a);
    cls.def("getCentroidPointNormal", &Class::getCentroidPointNormal, "normal_arg"_a);
        
}

template <typename PointT>
void defineSegmentationSupervoxelClustering(py::module &m, std::string const & suffix) {
    using Class = pcl::SupervoxelClustering<PointT>;
    using LeafContainerT = Class::LeafContainerT;
    using LeafVectorT = Class::LeafVectorT;
    using PointCloudT = Class::PointCloudT;
    using NormalCloudT = Class::NormalCloudT;
    using OctreeAdjacencyT = Class::OctreeAdjacencyT;
    using OctreeSearchT = Class::OctreeSearchT;
    using KdTreeT = Class::KdTreeT;
    using IndicesPtr = Class::IndicesPtr;
    using VoxelAdjacencyList = Class::VoxelAdjacencyList;
    using VoxelID = Class::VoxelID;
    using EdgeID = Class::EdgeID;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<float, float>(), "voxel_resolution"_a, "seed_resolution"_a);
    cls.def(py::init<float, float, bool>(), "voxel_resolution"_a, "seed_resolution"_a, ""_a);
    cls.def("extract", &Class::extract, "supervoxel_clusters"_a);
    cls.def("refineSupervoxels", &Class::refineSupervoxels, "num_itr"_a, "supervoxel_clusters"_a);
    cls.def_static("makeSupervoxelNormalCloud", &Class::makeSupervoxelNormalCloud, "supervoxel_clusters"_a);
    cls.def("setVoxelResolution", &Class::setVoxelResolution, "resolution"_a);
    cls.def("setSeedResolution", &Class::setSeedResolution, "seed_resolution"_a);
    cls.def("setColorImportance", &Class::setColorImportance, "val"_a);
    cls.def("setSpatialImportance", &Class::setSpatialImportance, "val"_a);
    cls.def("setNormalImportance", &Class::setNormalImportance, "val"_a);
    cls.def("setUseSingleCameraTransform", &Class::setUseSingleCameraTransform, "val"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setNormalCloud", &Class::setNormalCloud, "normal_cloud"_a);
    cls.def("getVoxelResolution", &Class::getVoxelResolution);
    cls.def("getSeedResolution", &Class::getSeedResolution);
    cls.def("getColoredCloud", &Class::getColoredCloud);
    cls.def("getVoxelCentroidCloud", &Class::getVoxelCentroidCloud);
    cls.def("getLabeledCloud", &Class::getLabeledCloud);
    cls.def("getColoredVoxelCloud", &Class::getColoredVoxelCloud);
    cls.def("getLabeledVoxelCloud", &Class::getLabeledVoxelCloud);
    cls.def("getSupervoxelAdjacencyList", &Class::getSupervoxelAdjacencyList, "adjacency_list_arg"_a);
    cls.def("getSupervoxelAdjacency", &Class::getSupervoxelAdjacency, "label_adjacency"_a);
    cls.def("getMaxLabel", &Class::getMaxLabel);
        
}

void defineSegmentationSupervoxelClusteringFunctions(py::module &m) {
}

void defineSegmentationSupervoxelClusteringClasses(py::module &sub_module) {
}