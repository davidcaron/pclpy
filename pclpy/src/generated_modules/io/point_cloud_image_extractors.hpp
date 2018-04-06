
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/point_cloud_image_extractors.h>

using namespace pcl::io;


template <typename PointT>
void defineIoPointCloudImageExtractor(py::module &m, std::string const & suffix) {
    using Class = io::PointCloudImageExtractor<PointT>;
    using PointCloud = Class::PointCloud;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("set_paint_na_ns_with_black", &Class::setPaintNaNsWithBlack);
    cls.def("extract", &Class::extract);
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromLabelField(py::module &m, std::string const & suffix) {
    using Class = io::PointCloudImageExtractorFromLabelField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PointCloudImageExtractor<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::ColorMode>(cls, "color_mode")
        .value("COLORS_MONO", Class::ColorMode::COLORS_MONO)
        .value("COLORS_RGB_RANDOM", Class::ColorMode::COLORS_RGB_RANDOM)
        .value("COLORS_RGB_GLASBEY", Class::ColorMode::COLORS_RGB_GLASBEY)
        .export_values();
    cls.def(py::init<Class::ColorMode>(), "color_mode"_a=Class::COLORS_MONO);
    cls.def("set_color_mode", &Class::setColorMode);
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromNormalField(py::module &m, std::string const & suffix) {
    using Class = io::PointCloudImageExtractorFromNormalField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PointCloudImageExtractor<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromRGBField(py::module &m, std::string const & suffix) {
    using Class = io::PointCloudImageExtractorFromRGBField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PointCloudImageExtractor<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorWithScaling(py::module &m, std::string const & suffix) {
    using Class = io::PointCloudImageExtractorWithScaling<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PointCloudImageExtractor<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::ScalingMethod>(cls, "scaling_method")
        .value("SCALING_NO", Class::ScalingMethod::SCALING_NO)
        .value("SCALING_FULL_RANGE", Class::ScalingMethod::SCALING_FULL_RANGE)
        .value("SCALING_FIXED_FACTOR", Class::ScalingMethod::SCALING_FIXED_FACTOR)
        .export_values();
    cls.def(py::init<std::string, Class::ScalingMethod>(), "field_name"_a, "scaling_method"_a);
    cls.def(py::init<std::string, float>(), "field_name"_a, "scaling_factor"_a);
    cls.def("set_scaling_method", &Class::setScalingMethod);
    cls.def("set_scaling_factor", &Class::setScalingFactor);
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromCurvatureField(py::module &m, std::string const & suffix) {
    using Class = io::PointCloudImageExtractorFromCurvatureField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PointCloudImageExtractorWithScaling<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromIntensityField(py::module &m, std::string const & suffix) {
    using Class = io::PointCloudImageExtractorFromIntensityField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PointCloudImageExtractorWithScaling<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromZField(py::module &m, std::string const & suffix) {
    using Class = io::PointCloudImageExtractorFromZField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PointCloudImageExtractorWithScaling<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

void defineIoPointCloudImageExtractorsClasses(py::module &sub_module) {
}