
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/segmentation/lccp_segmentation.h>



template <typename PointT>
void defineSegmentationLCCPSegmentation(py::module &m, std::string const & suffix) {
    using Class = pcl::LCCPSegmentation<PointT>;
    using SupervoxelAdjacencyList = Class::SupervoxelAdjacencyList;
    using VertexIterator = Class::VertexIterator;
    using AdjacencyIterator = Class::AdjacencyIterator;
    using VertexID = Class::VertexID;
    using EdgeIterator = Class::EdgeIterator;
    using OutEdgeIterator = Class::OutEdgeIterator;
    using EdgeID = Class::EdgeID;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("reset", &Class::reset);
    cls.def("segment", py::overload_cast<> (&Class::segment));
    cls.def("relabelCloud", &Class::relabelCloud, "labeled_cloud_arg"_a);
    cls.def("setInputSupervoxels", &Class::setInputSupervoxels, "supervoxel_clusters_arg"_a, "label_adjacency_arg"_a);
    cls.def("setConcavityToleranceThreshold", &Class::setConcavityToleranceThreshold, "concavity_tolerance_threshold_arg"_a);
    cls.def("setSmoothnessCheck", &Class::setSmoothnessCheck, "use_smoothness_check_arg"_a, "voxel_res_arg"_a, "seed_res_arg"_a, "smoothness_threshold_arg"_a=0.1);
    cls.def("setSanityCheck", &Class::setSanityCheck, "use_sanity_criterion_arg"_a);
    cls.def("setKFactor", &Class::setKFactor, "k_factor_arg"_a);
    cls.def("setMinSegmentSize", &Class::setMinSegmentSize, "min_segment_size_arg"_a);
    cls.def("getSegmentToSupervoxelMap", &Class::getSegmentToSupervoxelMap, "segment_supervoxel_map_arg"_a);
    cls.def("getSupervoxelToSegmentMap", &Class::getSupervoxelToSegmentMap, "supervoxel_segment_map_arg"_a);
    cls.def("getSegmentAdjacencyMap", &Class::getSegmentAdjacencyMap, "segment_adjacency_map_arg"_a);
    cls.def("getConcavityToleranceThreshold", &Class::getConcavityToleranceThreshold);
    cls.def("getSVAdjacencyList", &Class::getSVAdjacencyList, "adjacency_list_arg"_a);
        
}

void defineSegmentationLccpSegmentationFunctions(py::module &m) {
}

void defineSegmentationLccpSegmentationClasses(py::module &sub_module) {
    py::module sub_module_LCCPSegmentation = sub_module.def_submodule("LCCPSegmentation", "Submodule LCCPSegmentation");
    defineSegmentationLCCPSegmentation<pcl::PointXYZ>(sub_module_LCCPSegmentation, "PointXYZ");
    defineSegmentationLCCPSegmentation<pcl::PointXYZRGB>(sub_module_LCCPSegmentation, "PointXYZRGB");
    defineSegmentationLCCPSegmentation<pcl::PointXYZRGBA>(sub_module_LCCPSegmentation, "PointXYZRGBA");
    defineSegmentationLCCPSegmentation<pcl::PointXYZRGBNormal>(sub_module_LCCPSegmentation, "PointXYZRGBNormal");
}