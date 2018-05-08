
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/segmentation/min_cut_segmentation.h>



template <typename PointT>
void defineSegmentationMinCutSegmentation(py::module &m, std::string const & suffix) {
    using Class = pcl::MinCutSegmentation<PointT>;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Traits = Class::Traits;
    using mGraph = Class::mGraph;
    using CapacityMap = Class::CapacityMap;
    using ReverseEdgeMap = Class::ReverseEdgeMap;
    using VertexDescriptor = Class::VertexDescriptor;
    using EdgeDescriptor = Class::EdgeDescriptor;
    using OutEdgeIterator = Class::OutEdgeIterator;
    using VertexIterator = Class::VertexIterator;
    using ResidualCapacityMap = Class::ResidualCapacityMap;
    using IndexMap = Class::IndexMap;
    using InEdgeIterator = Class::InEdgeIterator;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("extract", &Class::extract, "clusters"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setSigma", &Class::setSigma, "sigma"_a);
    cls.def("setRadius", &Class::setRadius, "radius"_a);
    cls.def("setSourceWeight", &Class::setSourceWeight, "weight"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("setNumberOfNeighbours", &Class::setNumberOfNeighbours, "neighbour_number"_a);
    cls.def("setForegroundPoints", &Class::setForegroundPoints, "foreground_points"_a);
    cls.def("setBackgroundPoints", &Class::setBackgroundPoints, "background_points"_a);
    cls.def("getSigma", &Class::getSigma);
    cls.def("getRadius", &Class::getRadius);
    cls.def("getSourceWeight", &Class::getSourceWeight);
    cls.def("getSearchMethod", &Class::getSearchMethod);
    cls.def("getNumberOfNeighbours", &Class::getNumberOfNeighbours);
    cls.def("getForegroundPoints", &Class::getForegroundPoints);
    cls.def("getBackgroundPoints", &Class::getBackgroundPoints);
    cls.def("getMaxFlow", &Class::getMaxFlow);
    cls.def("getGraph", &Class::getGraph);
    cls.def("getColoredCloud", &Class::getColoredCloud);
        
}

void defineSegmentationMinCutSegmentationFunctions(py::module &m) {
}

void defineSegmentationMinCutSegmentationClasses(py::module &sub_module) {
    py::module sub_module_MinCutSegmentation = sub_module.def_submodule("MinCutSegmentation", "Submodule MinCutSegmentation");
    defineSegmentationMinCutSegmentation<pcl::InterestPoint>(sub_module_MinCutSegmentation, "InterestPoint");
    defineSegmentationMinCutSegmentation<pcl::PointDEM>(sub_module_MinCutSegmentation, "PointDEM");
    defineSegmentationMinCutSegmentation<pcl::PointNormal>(sub_module_MinCutSegmentation, "PointNormal");
    defineSegmentationMinCutSegmentation<pcl::PointSurfel>(sub_module_MinCutSegmentation, "PointSurfel");
    defineSegmentationMinCutSegmentation<pcl::PointWithRange>(sub_module_MinCutSegmentation, "PointWithRange");
    defineSegmentationMinCutSegmentation<pcl::PointWithScale>(sub_module_MinCutSegmentation, "PointWithScale");
    defineSegmentationMinCutSegmentation<pcl::PointWithViewpoint>(sub_module_MinCutSegmentation, "PointWithViewpoint");
    defineSegmentationMinCutSegmentation<pcl::PointXYZ>(sub_module_MinCutSegmentation, "PointXYZ");
    defineSegmentationMinCutSegmentation<pcl::PointXYZHSV>(sub_module_MinCutSegmentation, "PointXYZHSV");
    defineSegmentationMinCutSegmentation<pcl::PointXYZI>(sub_module_MinCutSegmentation, "PointXYZI");
    defineSegmentationMinCutSegmentation<pcl::PointXYZINormal>(sub_module_MinCutSegmentation, "PointXYZINormal");
    defineSegmentationMinCutSegmentation<pcl::PointXYZL>(sub_module_MinCutSegmentation, "PointXYZL");
    defineSegmentationMinCutSegmentation<pcl::PointXYZLNormal>(sub_module_MinCutSegmentation, "PointXYZLNormal");
    defineSegmentationMinCutSegmentation<pcl::PointXYZRGB>(sub_module_MinCutSegmentation, "PointXYZRGB");
    defineSegmentationMinCutSegmentation<pcl::PointXYZRGBA>(sub_module_MinCutSegmentation, "PointXYZRGBA");
    defineSegmentationMinCutSegmentation<pcl::PointXYZRGBL>(sub_module_MinCutSegmentation, "PointXYZRGBL");
    defineSegmentationMinCutSegmentation<pcl::PointXYZRGBNormal>(sub_module_MinCutSegmentation, "PointXYZRGBNormal");
}