
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/visualization/registration_visualizer.h>



template<typename PointSource, typename PointTarget>
void defineVisualizationRegistrationVisualizer(py::module &m, std::string const & suffix) {
    using Class = pcl::RegistrationVisualizer<PointSource, PointTarget>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("startDisplay", &Class::startDisplay);
    cls.def("stopDisplay", &Class::stopDisplay);
    cls.def("updateIntermediateCloud", &Class::updateIntermediateCloud, "cloud_src"_a, "indices_src"_a, "cloud_tgt"_a, "indices_tgt"_a);
    cls.def("setRegistration", &Class::setRegistration, "registration"_a);
    cls.def("setMaximumDisplayedCorrespondences", &Class::setMaximumDisplayedCorrespondences, "maximum_displayed_correspondences"_a);
    cls.def("getMaximumDisplayedCorrespondences", &Class::getMaximumDisplayedCorrespondences);
        
}

void defineVisualizationRegistrationVisualizerFunctions(py::module &m) {
}

void defineVisualizationRegistrationVisualizerClasses(py::module &sub_module) {
}