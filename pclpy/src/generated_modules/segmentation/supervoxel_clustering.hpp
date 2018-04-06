
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/supervoxel_clustering.h>



template <typename PointT>
void defineSegmentationSupervoxel(py::module &m, std::string const & suffix) {
    using Class = Supervoxel<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_readonly("normal_", &Class::normal_);
    cls.def_readonly("centroid_", &Class::centroid_);
    cls.def_readonly("voxels_", &Class::voxels_);
    cls.def_readonly("normals_", &Class::normals_);
        
}

template <typename PointT>
void defineSegmentationSupervoxelClustering(py::module &m, std::string const & suffix) {
    using Class = SupervoxelClustering<PointT>;
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
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<float, float>(), "voxel_resolution"_a, "seed_resolution"_a);
    cls.def(py::init<float, float, bool>(), "voxel_resolution"_a, "seed_resolution"_a, ""_a);
    cls.def_property("voxel_resolution", &Class::getVoxelResolution, &Class::setVoxelResolution);
    cls.def_property("seed_resolution", &Class::getSeedResolution, &Class::setSeedResolution);
    cls.def("set_color_importance", &Class::setColorImportance);
    cls.def("set_spatial_importance", &Class::setSpatialImportance);
    cls.def("set_normal_importance", &Class::setNormalImportance);
    cls.def("set_use_single_camera_transform", &Class::setUseSingleCameraTransform);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("set_normal_cloud", &Class::setNormalCloud);
    cls.def("extract", &Class::extract);
    cls.def("refine_supervoxels", &Class::refineSupervoxels);
    cls.def("make_supervoxel_normal_cloud", &Class::makeSupervoxelNormalCloud);
        
}

void defineSegmentationSupervoxelClusteringClasses(py::module &sub_module) {
}