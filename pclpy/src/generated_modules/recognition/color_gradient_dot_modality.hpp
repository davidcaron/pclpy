
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/color_gradient_dot_modality.h>



template <typename PointInT>
void defineRecognitionColorGradientDOTModality(py::module &m, std::string const & suffix) {
    using Class = ColorGradientDOTModality<PointInT>;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, DOTModality, PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<size_t>(), "bin_size"_a);
    cls.def("set_gradient_magnitude_threshold", &Class::setGradientMagnitudeThreshold);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("compute_invariant_quantized_map", &Class::computeInvariantQuantizedMap);
    cls.def("process_input_data", &Class::processInputData);
        
}

void defineRecognitionColorGradientDotModalityClasses(py::module &sub_module) {
}