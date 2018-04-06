
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/stereo/stereo_matching.h>



void defineStereoStereoMatching(py::module &m) {
    using Class = StereoMatching;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "StereoMatching");
    cls.def("set_max_disparity", &Class::setMaxDisparity);
    cls.def("set_x_off", &Class::setXOffset);
    cls.def("set_ratio_filter", &Class::setRatioFilter);
    cls.def("set_peak_filter", &Class::setPeakFilter);
    cls.def("set_pre_processing", &Class::setPreProcessing);
    cls.def("set_left_right_check", &Class::setLeftRightCheck);
    cls.def("set_left_right_check_threshold", &Class::setLeftRightCheckThreshold);
    cls.def("compute", py::overload_cast<unsigned char *, unsigned char *, int, int> (&Class::compute));
    cls.def("compute", py::overload_cast<pcl::PointCloud<pcl::RGB> &, pcl::PointCloud<pcl::RGB> &> (&Class::compute));
    cls.def("median_filter", &Class::medianFilter);
    cls.def("get_point_cloud", py::overload_cast<float, float, float, float, pcl::PointCloud<pcl::PointXYZ>::Ptr> (&Class::getPointCloud));
    cls.def("get_point_cloud", py::overload_cast<float, float, float, float, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::RGB>::Ptr> (&Class::getPointCloud));
}

void defineStereoGrayStereoMatching(py::module &m) {
    using Class = GrayStereoMatching;
    py::class_<Class, StereoMatching, boost::shared_ptr<Class>> cls(m, "GrayStereoMatching");
    cls.def("compute", py::overload_cast<unsigned char *, unsigned char *, int, int> (&Class::compute));
    cls.def("compute", py::overload_cast<pcl::PointCloud<pcl::RGB> &, pcl::PointCloud<pcl::RGB> &> (&Class::compute));
}

void defineStereoAdaptiveCostSOStereoMatching(py::module &m) {
    using Class = AdaptiveCostSOStereoMatching;
    py::class_<Class, GrayStereoMatching, boost::shared_ptr<Class>> cls(m, "AdaptiveCostSOStereoMatching");
    cls.def("set_radius", &Class::setRadius);
    cls.def("set_gamma_s", &Class::setGammaS);
    cls.def("set_gamma_c", &Class::setGammaC);
    cls.def("set_smooth_weak", &Class::setSmoothWeak);
    cls.def("set_smooth_strong", &Class::setSmoothStrong);
}

void defineStereoBlockBasedStereoMatching(py::module &m) {
    using Class = BlockBasedStereoMatching;
    py::class_<Class, GrayStereoMatching, boost::shared_ptr<Class>> cls(m, "BlockBasedStereoMatching");
    cls.def("set_radius", &Class::setRadius);
}

void defineStereoStereoMatchingClasses(py::module &sub_module) {
    defineStereoStereoMatching(sub_module);
    defineStereoGrayStereoMatching(sub_module);
    defineStereoAdaptiveCostSOStereoMatching(sub_module);
    defineStereoBlockBasedStereoMatching(sub_module);
}