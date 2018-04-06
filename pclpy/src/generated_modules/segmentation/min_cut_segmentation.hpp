
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/min_cut_segmentation.h>



template <typename PointT>
void defineSegmentationMinCutSegmentation(py::module &m, std::string const & suffix) {
    using Class = MinCutSegmentation<PointT>;
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
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def_property("sigma", &Class::getSigma, &Class::setSigma);
    cls.def_property("radius", &Class::getRadius, &Class::setRadius);
    cls.def_property("source_weight", &Class::getSourceWeight, &Class::setSourceWeight);
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def_property("number_of_neighbours", &Class::getNumberOfNeighbours, &Class::setNumberOfNeighbours);
    cls.def_property("foreground_points", &Class::getForegroundPoints, &Class::setForegroundPoints);
    cls.def_property("background_points", &Class::getBackgroundPoints, &Class::setBackgroundPoints);
    cls.def("extract", &Class::extract);
        
}

void defineSegmentationMinCutSegmentationClasses(py::module &sub_module) {
    py::module sub_module_MinCutSegmentation = sub_module.def_submodule("MinCutSegmentation", "Submodule MinCutSegmentation");
    defineSegmentationMinCutSegmentation<InterestPoint>(sub_module_MinCutSegmentation, "InterestPoint");
    defineSegmentationMinCutSegmentation<PointDEM>(sub_module_MinCutSegmentation, "PointDEM");
    defineSegmentationMinCutSegmentation<PointNormal>(sub_module_MinCutSegmentation, "PointNormal");
    defineSegmentationMinCutSegmentation<PointSurfel>(sub_module_MinCutSegmentation, "PointSurfel");
    defineSegmentationMinCutSegmentation<PointWithRange>(sub_module_MinCutSegmentation, "PointWithRange");
    defineSegmentationMinCutSegmentation<PointWithScale>(sub_module_MinCutSegmentation, "PointWithScale");
    defineSegmentationMinCutSegmentation<PointWithViewpoint>(sub_module_MinCutSegmentation, "PointWithViewpoint");
    defineSegmentationMinCutSegmentation<PointXYZ>(sub_module_MinCutSegmentation, "PointXYZ");
    defineSegmentationMinCutSegmentation<PointXYZHSV>(sub_module_MinCutSegmentation, "PointXYZHSV");
    defineSegmentationMinCutSegmentation<PointXYZI>(sub_module_MinCutSegmentation, "PointXYZI");
    defineSegmentationMinCutSegmentation<PointXYZINormal>(sub_module_MinCutSegmentation, "PointXYZINormal");
    defineSegmentationMinCutSegmentation<PointXYZL>(sub_module_MinCutSegmentation, "PointXYZL");
    defineSegmentationMinCutSegmentation<PointXYZLNormal>(sub_module_MinCutSegmentation, "PointXYZLNormal");
    defineSegmentationMinCutSegmentation<PointXYZRGB>(sub_module_MinCutSegmentation, "PointXYZRGB");
    defineSegmentationMinCutSegmentation<PointXYZRGBA>(sub_module_MinCutSegmentation, "PointXYZRGBA");
    defineSegmentationMinCutSegmentation<PointXYZRGBL>(sub_module_MinCutSegmentation, "PointXYZRGBL");
    defineSegmentationMinCutSegmentation<PointXYZRGBNormal>(sub_module_MinCutSegmentation, "PointXYZRGBNormal");
}