
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/recognition/color_gradient_dot_modality.h>



template <typename PointInT>
void defineRecognitionColorGradientDOTModality(py::module &m, std::string const & suffix) {
    using Class = pcl::ColorGradientDOTModality<PointInT>;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::DOTModality, pcl::PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<size_t>(), "bin_size"_a);
    cls.def("computeInvariantQuantizedMap", &Class::computeInvariantQuantizedMap, "mask"_a, "region"_a);
    cls.def("processInputData", &Class::processInputData);
    cls.def("setGradientMagnitudeThreshold", &Class::setGradientMagnitudeThreshold, "threshold"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("getDominantQuantizedMap", &Class::getDominantQuantizedMap);
        
}

void defineRecognitionColorGradientDotModalityFunctions(py::module &m) {
}

void defineRecognitionColorGradientDotModalityClasses(py::module &sub_module) {
}