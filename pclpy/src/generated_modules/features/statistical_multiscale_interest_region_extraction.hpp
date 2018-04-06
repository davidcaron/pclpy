
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/statistical_multiscale_interest_region_extraction.h>



template <typename PointT>
void defineFeaturesStatisticalMultiscaleInterestRegionExtraction(py::module &m, std::string const & suffix) {
    using Class = StatisticalMultiscaleInterestRegionExtraction<PointT>;
    using IndicesPtr = Class::IndicesPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("scales_vector", &Class::getScalesVector, &Class::setScalesVector);
    cls.def("generate_cloud_graph", &Class::generateCloudGraph);
    cls.def("compute_regions_of_interest", &Class::computeRegionsOfInterest);
        
}

void defineFeaturesStatisticalMultiscaleInterestRegionExtractionClasses(py::module &sub_module) {
    py::module sub_module_StatisticalMultiscaleInterestRegionExtraction = sub_module.def_submodule("StatisticalMultiscaleInterestRegionExtraction", "Submodule StatisticalMultiscaleInterestRegionExtraction");
    defineFeaturesStatisticalMultiscaleInterestRegionExtraction<PointXYZ>(sub_module_StatisticalMultiscaleInterestRegionExtraction, "PointXYZ");
}