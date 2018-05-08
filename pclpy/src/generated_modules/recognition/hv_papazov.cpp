
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

#include <pcl/recognition/hv/hv_papazov.h>



template<typename ModelT, typename SceneT>
void defineRecognitionPapazovHV(py::module &m, std::string const & suffix) {
    using Class = pcl::PapazovHV<ModelT, SceneT>;
    py::class_<Class, pcl::HypothesisVerification<pcl::ModelT, pcl::SceneT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("verify", &Class::verify);
    cls.def("setConflictThreshold", &Class::setConflictThreshold, "t"_a);
    cls.def("setSupportThreshold", &Class::setSupportThreshold, "t"_a);
    cls.def("setPenaltyThreshold", &Class::setPenaltyThreshold, "t"_a);
        
}

void defineRecognitionHvPapazovFunctions(py::module &m) {
}

void defineRecognitionHvPapazovClasses(py::module &sub_module) {
}