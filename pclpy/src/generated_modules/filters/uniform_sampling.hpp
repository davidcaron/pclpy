
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/uniform_sampling.h>



template <typename PointT>
void defineFiltersUniformSampling(py::module &m, std::string const & suffix) {
    using Class = UniformSampling<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_radius_search", &Class::setRadiusSearch);
        
}

void defineFiltersUniformSamplingClasses(py::module &sub_module) {
    py::module sub_module_UniformSampling = sub_module.def_submodule("UniformSampling", "Submodule UniformSampling");
    defineFiltersUniformSampling<InterestPoint>(sub_module_UniformSampling, "InterestPoint");
    defineFiltersUniformSampling<PointDEM>(sub_module_UniformSampling, "PointDEM");
    defineFiltersUniformSampling<PointNormal>(sub_module_UniformSampling, "PointNormal");
    defineFiltersUniformSampling<PointSurfel>(sub_module_UniformSampling, "PointSurfel");
    defineFiltersUniformSampling<PointWithRange>(sub_module_UniformSampling, "PointWithRange");
    defineFiltersUniformSampling<PointWithScale>(sub_module_UniformSampling, "PointWithScale");
    defineFiltersUniformSampling<PointWithViewpoint>(sub_module_UniformSampling, "PointWithViewpoint");
    defineFiltersUniformSampling<PointXYZ>(sub_module_UniformSampling, "PointXYZ");
    defineFiltersUniformSampling<PointXYZHSV>(sub_module_UniformSampling, "PointXYZHSV");
    defineFiltersUniformSampling<PointXYZI>(sub_module_UniformSampling, "PointXYZI");
    defineFiltersUniformSampling<PointXYZINormal>(sub_module_UniformSampling, "PointXYZINormal");
    defineFiltersUniformSampling<PointXYZL>(sub_module_UniformSampling, "PointXYZL");
    defineFiltersUniformSampling<PointXYZLNormal>(sub_module_UniformSampling, "PointXYZLNormal");
    defineFiltersUniformSampling<PointXYZRGB>(sub_module_UniformSampling, "PointXYZRGB");
    defineFiltersUniformSampling<PointXYZRGBA>(sub_module_UniformSampling, "PointXYZRGBA");
    defineFiltersUniformSampling<PointXYZRGBL>(sub_module_UniformSampling, "PointXYZRGBL");
    defineFiltersUniformSampling<PointXYZRGBNormal>(sub_module_UniformSampling, "PointXYZRGBNormal");
}