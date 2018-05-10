
#include <pcl/tracking/distance_coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingDistanceCoherence(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::DistanceCoherence<PointInT>;
    py::class_<Class, pcl::tracking::PointCoherence<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setWeight", &Class::setWeight, "weight"_a);
    cls.def("getWeight", &Class::getWeight);
        
}

void defineTrackingDistanceCoherenceFunctions(py::module &m) {
}

void defineTrackingDistanceCoherenceClasses(py::module &sub_module) {
    py::module sub_module_DistanceCoherence = sub_module.def_submodule("DistanceCoherence", "Submodule DistanceCoherence");
    defineTrackingDistanceCoherence<pcl::InterestPoint>(sub_module_DistanceCoherence, "InterestPoint");
    defineTrackingDistanceCoherence<pcl::PointDEM>(sub_module_DistanceCoherence, "PointDEM");
    defineTrackingDistanceCoherence<pcl::PointNormal>(sub_module_DistanceCoherence, "PointNormal");
    defineTrackingDistanceCoherence<pcl::PointSurfel>(sub_module_DistanceCoherence, "PointSurfel");
    defineTrackingDistanceCoherence<pcl::PointWithRange>(sub_module_DistanceCoherence, "PointWithRange");
    defineTrackingDistanceCoherence<pcl::PointWithScale>(sub_module_DistanceCoherence, "PointWithScale");
    defineTrackingDistanceCoherence<pcl::PointWithViewpoint>(sub_module_DistanceCoherence, "PointWithViewpoint");
    defineTrackingDistanceCoherence<pcl::PointXYZ>(sub_module_DistanceCoherence, "PointXYZ");
    defineTrackingDistanceCoherence<pcl::PointXYZHSV>(sub_module_DistanceCoherence, "PointXYZHSV");
    defineTrackingDistanceCoherence<pcl::PointXYZI>(sub_module_DistanceCoherence, "PointXYZI");
    defineTrackingDistanceCoherence<pcl::PointXYZINormal>(sub_module_DistanceCoherence, "PointXYZINormal");
    defineTrackingDistanceCoherence<pcl::PointXYZL>(sub_module_DistanceCoherence, "PointXYZL");
    defineTrackingDistanceCoherence<pcl::PointXYZLNormal>(sub_module_DistanceCoherence, "PointXYZLNormal");
    defineTrackingDistanceCoherence<pcl::PointXYZRGB>(sub_module_DistanceCoherence, "PointXYZRGB");
    defineTrackingDistanceCoherence<pcl::PointXYZRGBA>(sub_module_DistanceCoherence, "PointXYZRGBA");
    defineTrackingDistanceCoherence<pcl::PointXYZRGBL>(sub_module_DistanceCoherence, "PointXYZRGBL");
    defineTrackingDistanceCoherence<pcl::PointXYZRGBNormal>(sub_module_DistanceCoherence, "PointXYZRGBNormal");
}