
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/filters/statistical_outlier_removal.h>



template<typename PointT>
void defineFiltersStatisticalOutlierRemoval(py::module &m, std::string const & suffix) {
    using Class = pcl::StatisticalOutlierRemoval<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def("setMeanK", &Class::setMeanK, "nr_k"_a);
    cls.def("setStddevMulThresh", &Class::setStddevMulThresh, "stddev_mult"_a);
    cls.def("getMeanK", &Class::getMeanK);
    cls.def("getStddevMulThresh", &Class::getStddevMulThresh);
        
}

void defineFiltersStatisticalOutlierRemovalFunctions(py::module &m) {
}

void defineFiltersStatisticalOutlierRemovalClasses(py::module &sub_module) {
    py::module sub_module_StatisticalOutlierRemoval = sub_module.def_submodule("StatisticalOutlierRemoval", "Submodule StatisticalOutlierRemoval");
    defineFiltersStatisticalOutlierRemoval<pcl::InterestPoint>(sub_module_StatisticalOutlierRemoval, "InterestPoint");
    defineFiltersStatisticalOutlierRemoval<pcl::PointDEM>(sub_module_StatisticalOutlierRemoval, "PointDEM");
    defineFiltersStatisticalOutlierRemoval<pcl::PointNormal>(sub_module_StatisticalOutlierRemoval, "PointNormal");
    defineFiltersStatisticalOutlierRemoval<pcl::PointSurfel>(sub_module_StatisticalOutlierRemoval, "PointSurfel");
    defineFiltersStatisticalOutlierRemoval<pcl::PointWithRange>(sub_module_StatisticalOutlierRemoval, "PointWithRange");
    defineFiltersStatisticalOutlierRemoval<pcl::PointWithScale>(sub_module_StatisticalOutlierRemoval, "PointWithScale");
    defineFiltersStatisticalOutlierRemoval<pcl::PointWithViewpoint>(sub_module_StatisticalOutlierRemoval, "PointWithViewpoint");
    defineFiltersStatisticalOutlierRemoval<pcl::PointXYZ>(sub_module_StatisticalOutlierRemoval, "PointXYZ");
    defineFiltersStatisticalOutlierRemoval<pcl::PointXYZHSV>(sub_module_StatisticalOutlierRemoval, "PointXYZHSV");
    defineFiltersStatisticalOutlierRemoval<pcl::PointXYZI>(sub_module_StatisticalOutlierRemoval, "PointXYZI");
    defineFiltersStatisticalOutlierRemoval<pcl::PointXYZINormal>(sub_module_StatisticalOutlierRemoval, "PointXYZINormal");
    defineFiltersStatisticalOutlierRemoval<pcl::PointXYZL>(sub_module_StatisticalOutlierRemoval, "PointXYZL");
    defineFiltersStatisticalOutlierRemoval<pcl::PointXYZLNormal>(sub_module_StatisticalOutlierRemoval, "PointXYZLNormal");
    defineFiltersStatisticalOutlierRemoval<pcl::PointXYZRGB>(sub_module_StatisticalOutlierRemoval, "PointXYZRGB");
    defineFiltersStatisticalOutlierRemoval<pcl::PointXYZRGBA>(sub_module_StatisticalOutlierRemoval, "PointXYZRGBA");
    defineFiltersStatisticalOutlierRemoval<pcl::PointXYZRGBL>(sub_module_StatisticalOutlierRemoval, "PointXYZRGBL");
    defineFiltersStatisticalOutlierRemoval<pcl::PointXYZRGBNormal>(sub_module_StatisticalOutlierRemoval, "PointXYZRGBNormal");
}