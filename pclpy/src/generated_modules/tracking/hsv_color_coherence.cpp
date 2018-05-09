
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/tracking/hsv_color_coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingHSVColorCoherence(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::HSVColorCoherence<PointInT>;
    py::class_<Class, pcl::tracking::PointCoherence<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setWeight", &Class::setWeight, "weight"_a);
    cls.def("setHWeight", &Class::setHWeight, "weight"_a);
    cls.def("setSWeight", &Class::setSWeight, "weight"_a);
    cls.def("setVWeight", &Class::setVWeight, "weight"_a);
    cls.def("getWeight", &Class::getWeight);
    cls.def("getHWeight", &Class::getHWeight);
    cls.def("getSWeight", &Class::getSWeight);
    cls.def("getVWeight", &Class::getVWeight);
        
}

void defineTrackingHsvColorCoherenceFunctions(py::module &m) {
}

void defineTrackingHsvColorCoherenceClasses(py::module &sub_module) {
    py::module sub_module_HSVColorCoherence = sub_module.def_submodule("HSVColorCoherence", "Submodule HSVColorCoherence");
    defineTrackingHSVColorCoherence<pcl::PointXYZRGB>(sub_module_HSVColorCoherence, "PointXYZRGB");
    defineTrackingHSVColorCoherence<pcl::PointXYZRGBA>(sub_module_HSVColorCoherence, "PointXYZRGBA");
    defineTrackingHSVColorCoherence<pcl::PointXYZRGBNormal>(sub_module_HSVColorCoherence, "PointXYZRGBNormal");
}