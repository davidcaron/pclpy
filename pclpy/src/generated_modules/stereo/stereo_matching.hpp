
#include <pcl/stereo/stereo_matching.h>



void defineStereoStereoMatching(py::module &m) {
    using Class = pcl::StereoMatching;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "StereoMatching");
    cls.def("compute", py::overload_cast<unsigned char *, unsigned char *, int, int> (&Class::compute), "ref_img"_a, "trg_img"_a, "width"_a, "height"_a);
    cls.def("compute", py::overload_cast<pcl::PointCloud<pcl::RGB> &, pcl::PointCloud<pcl::RGB> &> (&Class::compute), "ref"_a, "trg"_a);
    cls.def("medianFilter", &Class::medianFilter, "radius"_a);
    cls.def("setMaxDisparity", &Class::setMaxDisparity, "max_disp"_a);
    cls.def("setXOffset", &Class::setXOffset, "x_off"_a);
    cls.def("setRatioFilter", &Class::setRatioFilter, "ratio_filter"_a);
    cls.def("setPeakFilter", &Class::setPeakFilter, "peak_filter"_a);
    cls.def("setPreProcessing", &Class::setPreProcessing, "is_pre_proc"_a);
    cls.def("setLeftRightCheck", &Class::setLeftRightCheck, "is_lr_check"_a);
    cls.def("setLeftRightCheckThreshold", &Class::setLeftRightCheckThreshold, "lr_check_th"_a);
    cls.def("getPointCloud", py::overload_cast<float, float, float, float, pcl::PointCloud<pcl::PointXYZ>::Ptr> (&Class::getPointCloud), "u_c"_a, "v_c"_a, "focal"_a, "baseline"_a, "cloud"_a);
    cls.def("getPointCloud", py::overload_cast<float, float, float, float, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::RGB>::Ptr> (&Class::getPointCloud), "u_c"_a, "v_c"_a, "focal"_a, "baseline"_a, "cloud"_a, "texture"_a);
    cls.def("getVisualMap", &Class::getVisualMap, "vMap"_a);
}

void defineStereoGrayStereoMatching(py::module &m) {
    using Class = pcl::GrayStereoMatching;
    py::class_<Class, pcl::StereoMatching, boost::shared_ptr<Class>> cls(m, "GrayStereoMatching");
    cls.def("compute", py::overload_cast<unsigned char *, unsigned char *, int, int> (&Class::compute), "ref_img"_a, "trg_img"_a, "width"_a, "height"_a);
    cls.def("compute", py::overload_cast<pcl::PointCloud<pcl::RGB> &, pcl::PointCloud<pcl::RGB> &> (&Class::compute), "ref"_a, "trg"_a);
}

void defineStereoAdaptiveCostSOStereoMatching(py::module &m) {
    using Class = pcl::AdaptiveCostSOStereoMatching;
    py::class_<Class, pcl::GrayStereoMatching, boost::shared_ptr<Class>> cls(m, "AdaptiveCostSOStereoMatching");
    cls.def("setRadius", &Class::setRadius, "radius"_a);
    cls.def("setGammaS", &Class::setGammaS, "gamma_s"_a);
    cls.def("setGammaC", &Class::setGammaC, "gamma_c"_a);
    cls.def("setSmoothWeak", &Class::setSmoothWeak, "smoothness_weak"_a);
    cls.def("setSmoothStrong", &Class::setSmoothStrong, "smoothness_strong"_a);
}

void defineStereoBlockBasedStereoMatching(py::module &m) {
    using Class = pcl::BlockBasedStereoMatching;
    py::class_<Class, pcl::GrayStereoMatching, boost::shared_ptr<Class>> cls(m, "BlockBasedStereoMatching");
    cls.def("setRadius", &Class::setRadius, "radius"_a);
}

void defineStereoStereoMatchingFunctions(py::module &m) {
}

void defineStereoStereoMatchingClasses(py::module &sub_module) {
    defineStereoStereoMatching(sub_module);
    defineStereoGrayStereoMatching(sub_module);
    defineStereoAdaptiveCostSOStereoMatching(sub_module);
    defineStereoBlockBasedStereoMatching(sub_module);
}