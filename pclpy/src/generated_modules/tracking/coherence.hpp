
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingPointCloudCoherence(py::module &m, std::string const & suffix) {
    using Class = tracking::PointCloudCoherence<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCoherencePtr = Class::PointCoherencePtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("point_coherences", &Class::getPointCoherences, &Class::setPointCoherences);
    cls.def("set_target_cloud", &Class::setTargetCloud);
    cls.def("compute", &Class::compute);
    cls.def("init_compute", &Class::initCompute);
    cls.def("add_point_coherence", &Class::addPointCoherence);
        
}

template <typename PointInT>
void defineTrackingPointCoherence(py::module &m, std::string const & suffix) {
    using Class = tracking::PointCoherence<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("compute", &Class::compute);
        
}

void defineTrackingCoherenceClasses(py::module &sub_module) {
    py::module sub_module_PointCloudCoherence = sub_module.def_submodule("PointCloudCoherence", "Submodule PointCloudCoherence");
    defineTrackingPointCloudCoherence<InterestPoint>(sub_module_PointCloudCoherence, "InterestPoint");
    defineTrackingPointCloudCoherence<PointDEM>(sub_module_PointCloudCoherence, "PointDEM");
    defineTrackingPointCloudCoherence<PointNormal>(sub_module_PointCloudCoherence, "PointNormal");
    defineTrackingPointCloudCoherence<PointSurfel>(sub_module_PointCloudCoherence, "PointSurfel");
    defineTrackingPointCloudCoherence<PointWithRange>(sub_module_PointCloudCoherence, "PointWithRange");
    defineTrackingPointCloudCoherence<PointWithScale>(sub_module_PointCloudCoherence, "PointWithScale");
    defineTrackingPointCloudCoherence<PointWithViewpoint>(sub_module_PointCloudCoherence, "PointWithViewpoint");
    defineTrackingPointCloudCoherence<PointXYZ>(sub_module_PointCloudCoherence, "PointXYZ");
    defineTrackingPointCloudCoherence<PointXYZHSV>(sub_module_PointCloudCoherence, "PointXYZHSV");
    defineTrackingPointCloudCoherence<PointXYZI>(sub_module_PointCloudCoherence, "PointXYZI");
    defineTrackingPointCloudCoherence<PointXYZINormal>(sub_module_PointCloudCoherence, "PointXYZINormal");
    defineTrackingPointCloudCoherence<PointXYZL>(sub_module_PointCloudCoherence, "PointXYZL");
    defineTrackingPointCloudCoherence<PointXYZLNormal>(sub_module_PointCloudCoherence, "PointXYZLNormal");
    defineTrackingPointCloudCoherence<PointXYZRGB>(sub_module_PointCloudCoherence, "PointXYZRGB");
    defineTrackingPointCloudCoherence<PointXYZRGBA>(sub_module_PointCloudCoherence, "PointXYZRGBA");
    defineTrackingPointCloudCoherence<PointXYZRGBL>(sub_module_PointCloudCoherence, "PointXYZRGBL");
    defineTrackingPointCloudCoherence<PointXYZRGBNormal>(sub_module_PointCloudCoherence, "PointXYZRGBNormal");
    py::module sub_module_PointCoherence = sub_module.def_submodule("PointCoherence", "Submodule PointCoherence");
    defineTrackingPointCoherence<InterestPoint>(sub_module_PointCoherence, "InterestPoint");
    defineTrackingPointCoherence<Normal>(sub_module_PointCoherence, "Normal");
    defineTrackingPointCoherence<PointDEM>(sub_module_PointCoherence, "PointDEM");
    defineTrackingPointCoherence<PointNormal>(sub_module_PointCoherence, "PointNormal");
    defineTrackingPointCoherence<PointSurfel>(sub_module_PointCoherence, "PointSurfel");
    defineTrackingPointCoherence<PointWithRange>(sub_module_PointCoherence, "PointWithRange");
    defineTrackingPointCoherence<PointWithScale>(sub_module_PointCoherence, "PointWithScale");
    defineTrackingPointCoherence<PointWithViewpoint>(sub_module_PointCoherence, "PointWithViewpoint");
    defineTrackingPointCoherence<PointXYZ>(sub_module_PointCoherence, "PointXYZ");
    defineTrackingPointCoherence<PointXYZHSV>(sub_module_PointCoherence, "PointXYZHSV");
    defineTrackingPointCoherence<PointXYZI>(sub_module_PointCoherence, "PointXYZI");
    defineTrackingPointCoherence<PointXYZINormal>(sub_module_PointCoherence, "PointXYZINormal");
    defineTrackingPointCoherence<PointXYZL>(sub_module_PointCoherence, "PointXYZL");
    defineTrackingPointCoherence<PointXYZLNormal>(sub_module_PointCoherence, "PointXYZLNormal");
    defineTrackingPointCoherence<PointXYZRGB>(sub_module_PointCoherence, "PointXYZRGB");
    defineTrackingPointCoherence<PointXYZRGBA>(sub_module_PointCoherence, "PointXYZRGBA");
    defineTrackingPointCoherence<PointXYZRGBL>(sub_module_PointCoherence, "PointXYZRGBL");
    defineTrackingPointCoherence<PointXYZRGBNormal>(sub_module_PointCoherence, "PointXYZRGBNormal");
}