
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/hsv_color_coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingHSVColorCoherence(py::module &m, std::string const & suffix) {
    using Class = tracking::HSVColorCoherence<PointInT>;
    py::class_<Class, PointCoherence<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("weight", &Class::getWeight, &Class::setWeight);
    cls.def_property("h_weight", &Class::getHWeight, &Class::setHWeight);
    cls.def_property("s_weight", &Class::getSWeight, &Class::setSWeight);
    cls.def_property("v_weight", &Class::getVWeight, &Class::setVWeight);
        
}

void defineTrackingHsvColorCoherenceClasses(py::module &sub_module) {
    py::module sub_module_HSVColorCoherence = sub_module.def_submodule("HSVColorCoherence", "Submodule HSVColorCoherence");
    defineTrackingHSVColorCoherence<PointXYZRGB>(sub_module_HSVColorCoherence, "PointXYZRGB");
    defineTrackingHSVColorCoherence<PointXYZRGBA>(sub_module_HSVColorCoherence, "PointXYZRGBA");
    defineTrackingHSVColorCoherence<PointXYZRGBNormal>(sub_module_HSVColorCoherence, "PointXYZRGBNormal");
}