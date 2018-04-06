
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/distance_coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingDistanceCoherence(py::module &m, std::string const & suffix) {
    using Class = tracking::DistanceCoherence<PointInT>;
    py::class_<Class, PointCoherence<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("weight", &Class::getWeight, &Class::setWeight);
        
}

void defineTrackingDistanceCoherenceClasses(py::module &sub_module) {
    py::module sub_module_DistanceCoherence = sub_module.def_submodule("DistanceCoherence", "Submodule DistanceCoherence");
    defineTrackingDistanceCoherence<InterestPoint>(sub_module_DistanceCoherence, "InterestPoint");
    defineTrackingDistanceCoherence<PointDEM>(sub_module_DistanceCoherence, "PointDEM");
    defineTrackingDistanceCoherence<PointNormal>(sub_module_DistanceCoherence, "PointNormal");
    defineTrackingDistanceCoherence<PointSurfel>(sub_module_DistanceCoherence, "PointSurfel");
    defineTrackingDistanceCoherence<PointWithRange>(sub_module_DistanceCoherence, "PointWithRange");
    defineTrackingDistanceCoherence<PointWithScale>(sub_module_DistanceCoherence, "PointWithScale");
    defineTrackingDistanceCoherence<PointWithViewpoint>(sub_module_DistanceCoherence, "PointWithViewpoint");
    defineTrackingDistanceCoherence<PointXYZ>(sub_module_DistanceCoherence, "PointXYZ");
    defineTrackingDistanceCoherence<PointXYZHSV>(sub_module_DistanceCoherence, "PointXYZHSV");
    defineTrackingDistanceCoherence<PointXYZI>(sub_module_DistanceCoherence, "PointXYZI");
    defineTrackingDistanceCoherence<PointXYZINormal>(sub_module_DistanceCoherence, "PointXYZINormal");
    defineTrackingDistanceCoherence<PointXYZL>(sub_module_DistanceCoherence, "PointXYZL");
    defineTrackingDistanceCoherence<PointXYZLNormal>(sub_module_DistanceCoherence, "PointXYZLNormal");
    defineTrackingDistanceCoherence<PointXYZRGB>(sub_module_DistanceCoherence, "PointXYZRGB");
    defineTrackingDistanceCoherence<PointXYZRGBA>(sub_module_DistanceCoherence, "PointXYZRGBA");
    defineTrackingDistanceCoherence<PointXYZRGBL>(sub_module_DistanceCoherence, "PointXYZRGBL");
    defineTrackingDistanceCoherence<PointXYZRGBNormal>(sub_module_DistanceCoherence, "PointXYZRGBNormal");
}