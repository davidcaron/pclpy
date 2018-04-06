
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/segment_differences.h>



template <typename PointT>
void defineSegmentationSegmentDifferences(py::module &m, std::string const & suffix) {
    using Class = SegmentDifferences<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("target_cloud", &Class::getTargetCloud, &Class::setTargetCloud);
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def_property("distance_threshold", &Class::getDistanceThreshold, &Class::setDistanceThreshold);
    cls.def("segment", py::overload_cast<PointCloud &> (&Class::segment));
        
}

void defineSegmentationSegmentDifferencesClasses(py::module &sub_module) {
    py::module sub_module_SegmentDifferences = sub_module.def_submodule("SegmentDifferences", "Submodule SegmentDifferences");
    defineSegmentationSegmentDifferences<InterestPoint>(sub_module_SegmentDifferences, "InterestPoint");
    defineSegmentationSegmentDifferences<PointDEM>(sub_module_SegmentDifferences, "PointDEM");
    defineSegmentationSegmentDifferences<PointNormal>(sub_module_SegmentDifferences, "PointNormal");
    defineSegmentationSegmentDifferences<PointSurfel>(sub_module_SegmentDifferences, "PointSurfel");
    defineSegmentationSegmentDifferences<PointWithRange>(sub_module_SegmentDifferences, "PointWithRange");
    defineSegmentationSegmentDifferences<PointWithScale>(sub_module_SegmentDifferences, "PointWithScale");
    defineSegmentationSegmentDifferences<PointWithViewpoint>(sub_module_SegmentDifferences, "PointWithViewpoint");
    defineSegmentationSegmentDifferences<PointXYZ>(sub_module_SegmentDifferences, "PointXYZ");
    defineSegmentationSegmentDifferences<PointXYZHSV>(sub_module_SegmentDifferences, "PointXYZHSV");
    defineSegmentationSegmentDifferences<PointXYZI>(sub_module_SegmentDifferences, "PointXYZI");
    defineSegmentationSegmentDifferences<PointXYZINormal>(sub_module_SegmentDifferences, "PointXYZINormal");
    defineSegmentationSegmentDifferences<PointXYZL>(sub_module_SegmentDifferences, "PointXYZL");
    defineSegmentationSegmentDifferences<PointXYZLNormal>(sub_module_SegmentDifferences, "PointXYZLNormal");
    defineSegmentationSegmentDifferences<PointXYZRGB>(sub_module_SegmentDifferences, "PointXYZRGB");
    defineSegmentationSegmentDifferences<PointXYZRGBA>(sub_module_SegmentDifferences, "PointXYZRGBA");
    defineSegmentationSegmentDifferences<PointXYZRGBL>(sub_module_SegmentDifferences, "PointXYZRGBL");
    defineSegmentationSegmentDifferences<PointXYZRGBNormal>(sub_module_SegmentDifferences, "PointXYZRGBNormal");
}