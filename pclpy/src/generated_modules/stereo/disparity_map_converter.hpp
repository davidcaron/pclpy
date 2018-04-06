
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/stereo/disparity_map_converter.h>



template <typename PointT>
void defineStereoDisparityMapConverter(py::module &m, std::string const & suffix) {
    using Class = DisparityMapConverter<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("image_center_x", &Class::getImageCenterX, &Class::setImageCenterX);
    cls.def_property("image_center_y", &Class::getImageCenterY, &Class::setImageCenterY);
    cls.def_property("focal_length", &Class::getFocalLength, &Class::setFocalLength);
    cls.def_property("baseline", &Class::getBaseline, &Class::setBaseline);
    cls.def_property("disparity_threshold_min", &Class::getDisparityThresholdMin, &Class::setDisparityThresholdMin);
    cls.def_property("disparity_threshold_max", &Class::getDisparityThresholdMax, &Class::setDisparityThresholdMax);
    cls.def_property("image", &Class::getImage, &Class::setImage);
    cls.def("load_disparity_map", py::overload_cast<const std::string &> (&Class::loadDisparityMap));
    cls.def("load_disparity_map", py::overload_cast<const std::string &, const size_t, const size_t> (&Class::loadDisparityMap));
    cls.def("compute", py::overload_cast<pcl::PointCloud<PointT> &> (&Class::compute));
    cls.def("set_disparity_map", py::overload_cast<const std::vector<float> &> (&Class::setDisparityMap));
    cls.def("set_disparity_map", py::overload_cast<const std::vector<float> &, const size_t, const size_t> (&Class::setDisparityMap));
        
}

void defineStereoDisparityMapConverterClasses(py::module &sub_module) {
    py::module sub_module_DisparityMapConverter = sub_module.def_submodule("DisparityMapConverter", "Submodule DisparityMapConverter");
    defineStereoDisparityMapConverter<InterestPoint>(sub_module_DisparityMapConverter, "InterestPoint");
    defineStereoDisparityMapConverter<PointDEM>(sub_module_DisparityMapConverter, "PointDEM");
    defineStereoDisparityMapConverter<PointNormal>(sub_module_DisparityMapConverter, "PointNormal");
    defineStereoDisparityMapConverter<PointSurfel>(sub_module_DisparityMapConverter, "PointSurfel");
    defineStereoDisparityMapConverter<PointWithRange>(sub_module_DisparityMapConverter, "PointWithRange");
    defineStereoDisparityMapConverter<PointWithScale>(sub_module_DisparityMapConverter, "PointWithScale");
    defineStereoDisparityMapConverter<PointWithViewpoint>(sub_module_DisparityMapConverter, "PointWithViewpoint");
    defineStereoDisparityMapConverter<PointXYZ>(sub_module_DisparityMapConverter, "PointXYZ");
    defineStereoDisparityMapConverter<PointXYZHSV>(sub_module_DisparityMapConverter, "PointXYZHSV");
    defineStereoDisparityMapConverter<PointXYZI>(sub_module_DisparityMapConverter, "PointXYZI");
    defineStereoDisparityMapConverter<PointXYZINormal>(sub_module_DisparityMapConverter, "PointXYZINormal");
    defineStereoDisparityMapConverter<PointXYZL>(sub_module_DisparityMapConverter, "PointXYZL");
    defineStereoDisparityMapConverter<PointXYZLNormal>(sub_module_DisparityMapConverter, "PointXYZLNormal");
    defineStereoDisparityMapConverter<PointXYZRGB>(sub_module_DisparityMapConverter, "PointXYZRGB");
    defineStereoDisparityMapConverter<PointXYZRGBA>(sub_module_DisparityMapConverter, "PointXYZRGBA");
    defineStereoDisparityMapConverter<PointXYZRGBL>(sub_module_DisparityMapConverter, "PointXYZRGBL");
    defineStereoDisparityMapConverter<PointXYZRGBNormal>(sub_module_DisparityMapConverter, "PointXYZRGBNormal");
}