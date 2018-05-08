
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

#include <pcl/filters/uniform_sampling.h>



template <typename PointT>
void defineFiltersUniformSampling(py::module &m, std::string const & suffix) {
    using Class = pcl::UniformSampling<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setRadiusSearch", &Class::setRadiusSearch, "radius"_a);
        
}

void defineFiltersUniformSamplingFunctions(py::module &m) {
}

void defineFiltersUniformSamplingClasses(py::module &sub_module) {
    py::module sub_module_UniformSampling = sub_module.def_submodule("UniformSampling", "Submodule UniformSampling");
    defineFiltersUniformSampling<pcl::InterestPoint>(sub_module_UniformSampling, "InterestPoint");
    defineFiltersUniformSampling<pcl::PointDEM>(sub_module_UniformSampling, "PointDEM");
    defineFiltersUniformSampling<pcl::PointNormal>(sub_module_UniformSampling, "PointNormal");
    defineFiltersUniformSampling<pcl::PointSurfel>(sub_module_UniformSampling, "PointSurfel");
    defineFiltersUniformSampling<pcl::PointWithRange>(sub_module_UniformSampling, "PointWithRange");
    defineFiltersUniformSampling<pcl::PointWithScale>(sub_module_UniformSampling, "PointWithScale");
    defineFiltersUniformSampling<pcl::PointWithViewpoint>(sub_module_UniformSampling, "PointWithViewpoint");
    defineFiltersUniformSampling<pcl::PointXYZ>(sub_module_UniformSampling, "PointXYZ");
    defineFiltersUniformSampling<pcl::PointXYZHSV>(sub_module_UniformSampling, "PointXYZHSV");
    defineFiltersUniformSampling<pcl::PointXYZI>(sub_module_UniformSampling, "PointXYZI");
    defineFiltersUniformSampling<pcl::PointXYZINormal>(sub_module_UniformSampling, "PointXYZINormal");
    defineFiltersUniformSampling<pcl::PointXYZL>(sub_module_UniformSampling, "PointXYZL");
    defineFiltersUniformSampling<pcl::PointXYZLNormal>(sub_module_UniformSampling, "PointXYZLNormal");
    defineFiltersUniformSampling<pcl::PointXYZRGB>(sub_module_UniformSampling, "PointXYZRGB");
    defineFiltersUniformSampling<pcl::PointXYZRGBA>(sub_module_UniformSampling, "PointXYZRGBA");
    defineFiltersUniformSampling<pcl::PointXYZRGBL>(sub_module_UniformSampling, "PointXYZRGBL");
    defineFiltersUniformSampling<pcl::PointXYZRGBNormal>(sub_module_UniformSampling, "PointXYZRGBNormal");
}