
#include <pcl/tracking/normal_coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingNormalCoherence(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::NormalCoherence<PointInT>;
    py::class_<Class, pcl::tracking::PointCoherence<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setWeight", &Class::setWeight, "weight"_a);
    cls.def("getWeight", &Class::getWeight);
        
}

void defineTrackingNormalCoherenceFunctions(py::module &m) {
}

void defineTrackingNormalCoherenceClasses(py::module &sub_module) {
    py::module sub_module_NormalCoherence = sub_module.def_submodule("NormalCoherence", "Submodule NormalCoherence");
    defineTrackingNormalCoherence<pcl::Normal>(sub_module_NormalCoherence, "Normal");
    defineTrackingNormalCoherence<pcl::PointNormal>(sub_module_NormalCoherence, "PointNormal");
    defineTrackingNormalCoherence<pcl::PointSurfel>(sub_module_NormalCoherence, "PointSurfel");
    defineTrackingNormalCoherence<pcl::PointXYZINormal>(sub_module_NormalCoherence, "PointXYZINormal");
    defineTrackingNormalCoherence<pcl::PointXYZLNormal>(sub_module_NormalCoherence, "PointXYZLNormal");
    defineTrackingNormalCoherence<pcl::PointXYZRGBNormal>(sub_module_NormalCoherence, "PointXYZRGBNormal");
}