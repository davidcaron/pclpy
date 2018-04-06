
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/normal_coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingNormalCoherence(py::module &m, std::string const & suffix) {
    using Class = tracking::NormalCoherence<PointInT>;
    py::class_<Class, PointCoherence<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("weight", &Class::getWeight, &Class::setWeight);
        
}

void defineTrackingNormalCoherenceClasses(py::module &sub_module) {
    py::module sub_module_NormalCoherence = sub_module.def_submodule("NormalCoherence", "Submodule NormalCoherence");
    defineTrackingNormalCoherence<Normal>(sub_module_NormalCoherence, "Normal");
    defineTrackingNormalCoherence<PointNormal>(sub_module_NormalCoherence, "PointNormal");
    defineTrackingNormalCoherence<PointSurfel>(sub_module_NormalCoherence, "PointSurfel");
    defineTrackingNormalCoherence<PointXYZINormal>(sub_module_NormalCoherence, "PointXYZINormal");
    defineTrackingNormalCoherence<PointXYZLNormal>(sub_module_NormalCoherence, "PointXYZLNormal");
    defineTrackingNormalCoherence<PointXYZRGBNormal>(sub_module_NormalCoherence, "PointXYZRGBNormal");
}