
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/lccp_segmentation.h>



template <typename PointT>
void defineSegmentationLCCPSegmentation(py::module &m, std::string const & suffix) {
    using Class = LCCPSegmentation<PointT>;
    using SupervoxelAdjacencyList = Class::SupervoxelAdjacencyList;
    using VertexIterator = Class::VertexIterator;
    using AdjacencyIterator = Class::AdjacencyIterator;
    using VertexID = Class::VertexID;
    using EdgeIterator = Class::EdgeIterator;
    using OutEdgeIterator = Class::OutEdgeIterator;
    using EdgeID = Class::EdgeID;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_input_supervoxels", &Class::setInputSupervoxels);
    cls.def_property("concavity_tolerance_threshold", &Class::getConcavityToleranceThreshold, &Class::setConcavityToleranceThreshold);
    cls.def("set_smoothness_check", &Class::setSmoothnessCheck);
    cls.def("set_sanity_check", &Class::setSanityCheck);
    cls.def("set_k_factor", &Class::setKFactor);
    cls.def("set_min_segment_size", &Class::setMinSegmentSize);
    cls.def("segment", py::overload_cast<> (&Class::segment));
    cls.def("reset", &Class::reset);
    cls.def("relabel_cloud", &Class::relabelCloud);
        
}

void defineSegmentationLccpSegmentationClasses(py::module &sub_module) {
    py::module sub_module_LCCPSegmentation = sub_module.def_submodule("LCCPSegmentation", "Submodule LCCPSegmentation");
    defineSegmentationLCCPSegmentation<PointXYZ>(sub_module_LCCPSegmentation, "PointXYZ");
    defineSegmentationLCCPSegmentation<PointXYZRGB>(sub_module_LCCPSegmentation, "PointXYZRGB");
    defineSegmentationLCCPSegmentation<PointXYZRGBA>(sub_module_LCCPSegmentation, "PointXYZRGBA");
    defineSegmentationLCCPSegmentation<PointXYZRGBNormal>(sub_module_LCCPSegmentation, "PointXYZRGBNormal");
}