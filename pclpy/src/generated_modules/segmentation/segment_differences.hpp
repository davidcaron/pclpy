
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/segment_differences.h>



template <typename PointT>
void defineSegmentationSegmentDifferences(py::module &m, std::string const & suffix) {
    using Class = pcl::SegmentDifferences<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("segment", py::overload_cast<PointCloud &> (&Class::segment), "output"_a);
    cls.def("setTargetCloud", &Class::setTargetCloud, "cloud"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("setDistanceThreshold", &Class::setDistanceThreshold, "sqr_threshold"_a);
    cls.def("getTargetCloud", &Class::getTargetCloud);
    cls.def("getSearchMethod", &Class::getSearchMethod);
    cls.def("getDistanceThreshold", &Class::getDistanceThreshold);
        
}

template <typename PointT>
void defineSegmentationSegmentDifferencesFunctions1(py::module &m) {
    m.def("getPointCloudDifference", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointCloud<PointT> &, double, const boost::shared_ptr<pcl::search::Search<PointT> > &, pcl::PointCloud<PointT> &> (&pcl::getPointCloudDifference<PointT>), "src"_a, "tgt"_a, "threshold"_a, "tree"_a, "output"_a);
}

void defineSegmentationSegmentDifferencesFunctions(py::module &m) {
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointXYZ>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointXYZI>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointXYZL>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointXYZRGBA>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointXYZRGB>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointXYZRGBL>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointXYZHSV>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::InterestPoint>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointNormal>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointXYZRGBNormal>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointXYZINormal>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointXYZLNormal>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointWithRange>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointWithViewpoint>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointWithScale>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointSurfel>(m);
    defineSegmentationSegmentDifferencesFunctions1<pcl::PointDEM>(m);
}

void defineSegmentationSegmentDifferencesClasses(py::module &sub_module) {
    py::module sub_module_SegmentDifferences = sub_module.def_submodule("SegmentDifferences", "Submodule SegmentDifferences");
    defineSegmentationSegmentDifferences<pcl::InterestPoint>(sub_module_SegmentDifferences, "InterestPoint");
    defineSegmentationSegmentDifferences<pcl::PointDEM>(sub_module_SegmentDifferences, "PointDEM");
    defineSegmentationSegmentDifferences<pcl::PointNormal>(sub_module_SegmentDifferences, "PointNormal");
    defineSegmentationSegmentDifferences<pcl::PointSurfel>(sub_module_SegmentDifferences, "PointSurfel");
    defineSegmentationSegmentDifferences<pcl::PointWithRange>(sub_module_SegmentDifferences, "PointWithRange");
    defineSegmentationSegmentDifferences<pcl::PointWithScale>(sub_module_SegmentDifferences, "PointWithScale");
    defineSegmentationSegmentDifferences<pcl::PointWithViewpoint>(sub_module_SegmentDifferences, "PointWithViewpoint");
    defineSegmentationSegmentDifferences<pcl::PointXYZ>(sub_module_SegmentDifferences, "PointXYZ");
    defineSegmentationSegmentDifferences<pcl::PointXYZHSV>(sub_module_SegmentDifferences, "PointXYZHSV");
    defineSegmentationSegmentDifferences<pcl::PointXYZI>(sub_module_SegmentDifferences, "PointXYZI");
    defineSegmentationSegmentDifferences<pcl::PointXYZINormal>(sub_module_SegmentDifferences, "PointXYZINormal");
    defineSegmentationSegmentDifferences<pcl::PointXYZL>(sub_module_SegmentDifferences, "PointXYZL");
    defineSegmentationSegmentDifferences<pcl::PointXYZLNormal>(sub_module_SegmentDifferences, "PointXYZLNormal");
    defineSegmentationSegmentDifferences<pcl::PointXYZRGB>(sub_module_SegmentDifferences, "PointXYZRGB");
    defineSegmentationSegmentDifferences<pcl::PointXYZRGBA>(sub_module_SegmentDifferences, "PointXYZRGBA");
    defineSegmentationSegmentDifferences<pcl::PointXYZRGBL>(sub_module_SegmentDifferences, "PointXYZRGBL");
    defineSegmentationSegmentDifferences<pcl::PointXYZRGBNormal>(sub_module_SegmentDifferences, "PointXYZRGBNormal");
    defineSegmentationSegmentDifferencesFunctions(sub_module);
}