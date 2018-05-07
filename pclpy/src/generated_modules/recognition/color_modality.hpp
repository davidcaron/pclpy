
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/recognition/color_modality.h>



template <typename PointInT>
void defineRecognitionColorModality(py::module &m, std::string const & suffix) {
    using Class = pcl::ColorModality<PointInT>;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::QuantizableModality, pcl::PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("extractFeatures", &Class::extractFeatures, "mask"_a, "nr_features"_a, "modalityIndex"_a, "features"_a);
    cls.def("processInputData", &Class::processInputData);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("getQuantizedMap", &Class::getQuantizedMap);
    cls.def("getSpreadedQuantizedMap", &Class::getSpreadedQuantizedMap);
        
}

void defineRecognitionColorModalityFunctions(py::module &m) {
}

void defineRecognitionColorModalityClasses(py::module &sub_module) {
}