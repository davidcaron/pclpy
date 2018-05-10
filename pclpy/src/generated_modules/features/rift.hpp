
#include <pcl/features/rift.h>



template <typename PointInT, typename GradientT, typename PointOutT>
void defineFeaturesRIFTEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::RIFTEstimation<PointInT, GradientT, PointOutT>;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudGradient = Class::PointCloudGradient;
    using PointCloudGradientPtr = Class::PointCloudGradientPtr;
    using PointCloudGradientConstPtr = Class::PointCloudGradientConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computeRIFT", &Class::computeRIFT, "cloud"_a, "gradient"_a, "p_idx"_a, "radius"_a, "indices"_a, "squared_distances"_a, "rift_descriptor"_a);
    cls.def("setInputGradient", &Class::setInputGradient, "gradient"_a);
    cls.def("setNrDistanceBins", &Class::setNrDistanceBins, "nr_distance_bins"_a);
    cls.def("setNrGradientBins", &Class::setNrGradientBins, "nr_gradient_bins"_a);
    cls.def("getInputGradient", &Class::getInputGradient);
    cls.def("getNrDistanceBins", &Class::getNrDistanceBins);
    cls.def("getNrGradientBins", &Class::getNrGradientBins);
        
}

void defineFeaturesRiftFunctions(py::module &m) {
}

void defineFeaturesRiftClasses(py::module &sub_module) {
    py::module sub_module_RIFTEstimation = sub_module.def_submodule("RIFTEstimation", "Submodule RIFTEstimation");
    defineFeaturesRIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, pcl::Histogram<32>>(sub_module_RIFTEstimation, "PointXYZI_IntensityGradient_Histogram<32>");
}