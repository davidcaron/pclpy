
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/rift.h>



template <typename PointInT, typename GradientT, typename PointOutT>
void defineFeaturesRIFTEstimation(py::module &m, std::string const & suffix) {
    using Class = RIFTEstimation<PointInT, GradientT, PointOutT>;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudGradient = Class::PointCloudGradient;
    using PointCloudGradientPtr = Class::PointCloudGradientPtr;
    using PointCloudGradientConstPtr = Class::PointCloudGradientConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("input_gradient", &Class::getInputGradient, &Class::setInputGradient);
    cls.def_property("nr_distance_bins", &Class::getNrDistanceBins, &Class::setNrDistanceBins);
    cls.def_property("nr_gradient_bins", &Class::getNrGradientBins, &Class::setNrGradientBins);
    cls.def("compute_rift", &Class::computeRIFT);
        
}

void defineFeaturesRiftClasses(py::module &sub_module) {
    py::module sub_module_RIFTEstimation = sub_module.def_submodule("RIFTEstimation", "Submodule RIFTEstimation");
    defineFeaturesRIFTEstimation<PointXYZI, IntensityGradient, Histogram<32>>(sub_module_RIFTEstimation, "PointXYZI_IntensityGradient_Histogram<32>");
}