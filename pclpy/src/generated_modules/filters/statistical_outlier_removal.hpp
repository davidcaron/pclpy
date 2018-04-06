
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/statistical_outlier_removal.h>



template<typename PointT>
void defineFiltersStatisticalOutlierRemoval(py::module &m, std::string const & suffix) {
    using Class = StatisticalOutlierRemoval<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def_property("mean_k", &Class::getMeanK, &Class::setMeanK);
    cls.def_property("stddev_mul_thresh", &Class::getStddevMulThresh, &Class::setStddevMulThresh);
        
}



void defineFiltersStatisticalOutlierRemovalClasses(py::module &sub_module) {
    py::module sub_module_StatisticalOutlierRemoval = sub_module.def_submodule("StatisticalOutlierRemoval", "Submodule StatisticalOutlierRemoval");
    defineFiltersStatisticalOutlierRemoval<InterestPoint>(sub_module_StatisticalOutlierRemoval, "InterestPoint");
    defineFiltersStatisticalOutlierRemoval<PointDEM>(sub_module_StatisticalOutlierRemoval, "PointDEM");
    defineFiltersStatisticalOutlierRemoval<PointNormal>(sub_module_StatisticalOutlierRemoval, "PointNormal");
    defineFiltersStatisticalOutlierRemoval<PointSurfel>(sub_module_StatisticalOutlierRemoval, "PointSurfel");
    defineFiltersStatisticalOutlierRemoval<PointWithRange>(sub_module_StatisticalOutlierRemoval, "PointWithRange");
    defineFiltersStatisticalOutlierRemoval<PointWithScale>(sub_module_StatisticalOutlierRemoval, "PointWithScale");
    defineFiltersStatisticalOutlierRemoval<PointWithViewpoint>(sub_module_StatisticalOutlierRemoval, "PointWithViewpoint");
    defineFiltersStatisticalOutlierRemoval<PointXYZ>(sub_module_StatisticalOutlierRemoval, "PointXYZ");
    defineFiltersStatisticalOutlierRemoval<PointXYZHSV>(sub_module_StatisticalOutlierRemoval, "PointXYZHSV");
    defineFiltersStatisticalOutlierRemoval<PointXYZI>(sub_module_StatisticalOutlierRemoval, "PointXYZI");
    defineFiltersStatisticalOutlierRemoval<PointXYZINormal>(sub_module_StatisticalOutlierRemoval, "PointXYZINormal");
    defineFiltersStatisticalOutlierRemoval<PointXYZL>(sub_module_StatisticalOutlierRemoval, "PointXYZL");
    defineFiltersStatisticalOutlierRemoval<PointXYZLNormal>(sub_module_StatisticalOutlierRemoval, "PointXYZLNormal");
    defineFiltersStatisticalOutlierRemoval<PointXYZRGB>(sub_module_StatisticalOutlierRemoval, "PointXYZRGB");
    defineFiltersStatisticalOutlierRemoval<PointXYZRGBA>(sub_module_StatisticalOutlierRemoval, "PointXYZRGBA");
    defineFiltersStatisticalOutlierRemoval<PointXYZRGBL>(sub_module_StatisticalOutlierRemoval, "PointXYZRGBL");
    defineFiltersStatisticalOutlierRemoval<PointXYZRGBNormal>(sub_module_StatisticalOutlierRemoval, "PointXYZRGBNormal");
}