
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/io/point_cloud_image_extractors.h>

using namespace pcl::io;


template <typename PointT>
void defineIoPointCloudImageExtractor(py::module &m, std::string const & suffix) {
    using Class = pcl::io::PointCloudImageExtractor<PointT>;
    using PointCloud = Class::PointCloud;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("extract", &Class::extract, "cloud"_a, "image"_a);
    cls.def("setPaintNaNsWithBlack", &Class::setPaintNaNsWithBlack, "flag"_a);
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromLabelField(py::module &m, std::string const & suffix) {
    using Class = pcl::io::PointCloudImageExtractorFromLabelField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::io::PointCloudImageExtractor<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::ColorMode>(cls, "ColorMode")
        .value("COLORS_MONO", Class::ColorMode::COLORS_MONO)
        .value("COLORS_RGB_RANDOM", Class::ColorMode::COLORS_RGB_RANDOM)
        .value("COLORS_RGB_GLASBEY", Class::ColorMode::COLORS_RGB_GLASBEY)
        .export_values();
    cls.def(py::init<Class::ColorMode>(), "color_mode"_a=Class::COLORS_MONO);
    cls.def("setColorMode", &Class::setColorMode, "color_mode"_a);
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromNormalField(py::module &m, std::string const & suffix) {
    using Class = pcl::io::PointCloudImageExtractorFromNormalField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::io::PointCloudImageExtractor<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromRGBField(py::module &m, std::string const & suffix) {
    using Class = pcl::io::PointCloudImageExtractorFromRGBField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::io::PointCloudImageExtractor<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorWithScaling(py::module &m, std::string const & suffix) {
    using Class = pcl::io::PointCloudImageExtractorWithScaling<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::io::PointCloudImageExtractor<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::ScalingMethod>(cls, "ScalingMethod")
        .value("SCALING_NO", Class::ScalingMethod::SCALING_NO)
        .value("SCALING_FULL_RANGE", Class::ScalingMethod::SCALING_FULL_RANGE)
        .value("SCALING_FIXED_FACTOR", Class::ScalingMethod::SCALING_FIXED_FACTOR)
        .export_values();
    cls.def(py::init<std::string, Class::ScalingMethod>(), "field_name"_a, "scaling_method"_a);
    cls.def(py::init<std::string, float>(), "field_name"_a, "scaling_factor"_a);
    cls.def("setScalingMethod", &Class::setScalingMethod, "scaling_method"_a);
    cls.def("setScalingFactor", &Class::setScalingFactor, "scaling_factor"_a);
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromCurvatureField(py::module &m, std::string const & suffix) {
    using Class = pcl::io::PointCloudImageExtractorFromCurvatureField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::io::PointCloudImageExtractorWithScaling<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromIntensityField(py::module &m, std::string const & suffix) {
    using Class = pcl::io::PointCloudImageExtractorFromIntensityField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::io::PointCloudImageExtractorWithScaling<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

template <typename PointT>
void defineIoPointCloudImageExtractorFromZField(py::module &m, std::string const & suffix) {
    using Class = pcl::io::PointCloudImageExtractorFromZField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::io::PointCloudImageExtractorWithScaling<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

void defineIoPointCloudImageExtractorsFunctions(py::module &m) {
}

void defineIoPointCloudImageExtractorsClasses(py::module &sub_module) {
}