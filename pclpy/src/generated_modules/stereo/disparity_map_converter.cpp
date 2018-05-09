
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/stereo/disparity_map_converter.h>



template <typename PointT>
void defineStereoDisparityMapConverter(py::module &m, std::string const & suffix) {
    using Class = pcl::DisparityMapConverter<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("loadDisparityMap", py::overload_cast<const std::string &> (&Class::loadDisparityMap), "file_name"_a);
    cls.def("loadDisparityMap", py::overload_cast<const std::string &, const size_t, const size_t> (&Class::loadDisparityMap), "file_name"_a, "width"_a, "height"_a);
    cls.def("compute", py::overload_cast<pcl::PointCloud<PointT> &> (&Class::compute), "out_cloud"_a);
    cls.def("setImageCenterX", &Class::setImageCenterX, "center_x"_a);
    cls.def("setImageCenterY", &Class::setImageCenterY, "center_y"_a);
    cls.def("setFocalLength", &Class::setFocalLength, "focal_length"_a);
    cls.def("setBaseline", &Class::setBaseline, "baseline"_a);
    cls.def("setDisparityThresholdMin", &Class::setDisparityThresholdMin, "disparity_threshold_min"_a);
    cls.def("setDisparityThresholdMax", &Class::setDisparityThresholdMax, "disparity_threshold_max"_a);
    cls.def("setImage", &Class::setImage, "image"_a);
    cls.def("setDisparityMap", py::overload_cast<const std::vector<float> &> (&Class::setDisparityMap), "disparity_map"_a);
    cls.def("setDisparityMap", py::overload_cast<const std::vector<float> &, const size_t, const size_t> (&Class::setDisparityMap), "disparity_map"_a, "width"_a, "height"_a);
    cls.def("getImageCenterX", &Class::getImageCenterX);
    cls.def("getImageCenterY", &Class::getImageCenterY);
    cls.def("getFocalLength", &Class::getFocalLength);
    cls.def("getBaseline", &Class::getBaseline);
    cls.def("getDisparityThresholdMin", &Class::getDisparityThresholdMin);
    cls.def("getDisparityThresholdMax", &Class::getDisparityThresholdMax);
    cls.def("getImage", &Class::getImage);
    cls.def("getDisparityMap", &Class::getDisparityMap);
        
}

void defineStereoDisparityMapConverterFunctions(py::module &m) {
}

void defineStereoDisparityMapConverterClasses(py::module &sub_module) {
    py::module sub_module_DisparityMapConverter = sub_module.def_submodule("DisparityMapConverter", "Submodule DisparityMapConverter");
    defineStereoDisparityMapConverter<pcl::InterestPoint>(sub_module_DisparityMapConverter, "InterestPoint");
    defineStereoDisparityMapConverter<pcl::PointDEM>(sub_module_DisparityMapConverter, "PointDEM");
    defineStereoDisparityMapConverter<pcl::PointNormal>(sub_module_DisparityMapConverter, "PointNormal");
    defineStereoDisparityMapConverter<pcl::PointSurfel>(sub_module_DisparityMapConverter, "PointSurfel");
    defineStereoDisparityMapConverter<pcl::PointWithRange>(sub_module_DisparityMapConverter, "PointWithRange");
    defineStereoDisparityMapConverter<pcl::PointWithScale>(sub_module_DisparityMapConverter, "PointWithScale");
    defineStereoDisparityMapConverter<pcl::PointWithViewpoint>(sub_module_DisparityMapConverter, "PointWithViewpoint");
    defineStereoDisparityMapConverter<pcl::PointXYZ>(sub_module_DisparityMapConverter, "PointXYZ");
    defineStereoDisparityMapConverter<pcl::PointXYZHSV>(sub_module_DisparityMapConverter, "PointXYZHSV");
    defineStereoDisparityMapConverter<pcl::PointXYZI>(sub_module_DisparityMapConverter, "PointXYZI");
    defineStereoDisparityMapConverter<pcl::PointXYZINormal>(sub_module_DisparityMapConverter, "PointXYZINormal");
    defineStereoDisparityMapConverter<pcl::PointXYZL>(sub_module_DisparityMapConverter, "PointXYZL");
    defineStereoDisparityMapConverter<pcl::PointXYZLNormal>(sub_module_DisparityMapConverter, "PointXYZLNormal");
    defineStereoDisparityMapConverter<pcl::PointXYZRGB>(sub_module_DisparityMapConverter, "PointXYZRGB");
    defineStereoDisparityMapConverter<pcl::PointXYZRGBA>(sub_module_DisparityMapConverter, "PointXYZRGBA");
    defineStereoDisparityMapConverter<pcl::PointXYZRGBL>(sub_module_DisparityMapConverter, "PointXYZRGBL");
    defineStereoDisparityMapConverter<pcl::PointXYZRGBNormal>(sub_module_DisparityMapConverter, "PointXYZRGBNormal");
}