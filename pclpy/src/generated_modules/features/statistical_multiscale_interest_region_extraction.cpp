
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/statistical_multiscale_interest_region_extraction.h>



template <typename PointT>
void defineFeaturesStatisticalMultiscaleInterestRegionExtraction(py::module &m, std::string const & suffix) {
    using Class = pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>;
    using IndicesPtr = Class::IndicesPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("generateCloudGraph", &Class::generateCloudGraph);
    cls.def("computeRegionsOfInterest", &Class::computeRegionsOfInterest, "rois"_a);
    cls.def("setScalesVector", &Class::setScalesVector, "scale_values"_a);
    cls.def("getScalesVector", &Class::getScalesVector);
        
}

void defineFeaturesStatisticalMultiscaleInterestRegionExtractionFunctions(py::module &m) {
}

void defineFeaturesStatisticalMultiscaleInterestRegionExtractionClasses(py::module &sub_module) {
    py::module sub_module_StatisticalMultiscaleInterestRegionExtraction = sub_module.def_submodule("StatisticalMultiscaleInterestRegionExtraction", "Submodule StatisticalMultiscaleInterestRegionExtraction");
    defineFeaturesStatisticalMultiscaleInterestRegionExtraction<pcl::PointXYZ>(sub_module_StatisticalMultiscaleInterestRegionExtraction, "PointXYZ");
}