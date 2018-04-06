
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/color_modality.h>



template <typename PointInT>
void defineRecognitionColorModality(py::module &m, std::string const & suffix) {
    using Class = ColorModality<PointInT>;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, QuantizableModality, PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("extract_features", &Class::extractFeatures);
    cls.def("process_input_data", &Class::processInputData);
        
}

void defineRecognitionColorModalityClasses(py::module &sub_module) {
}