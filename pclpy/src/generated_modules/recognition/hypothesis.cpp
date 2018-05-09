
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/recognition/ransac_based/hypothesis.h>

using namespace pcl::recognition;


void defineRecognitionHypothesisBase(py::module &m) {
    using Class = pcl::recognition::HypothesisBase;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "HypothesisBase");
    cls.def(py::init<ModelLibrary::Model*>(), "obj_model"_a);
    cls.def(py::init<ModelLibrary::Model*, float*>(), "obj_model"_a, "rigid_transform"_a);
    cls.def_readonly("rigid_transform_", &Class::rigid_transform_);
    cls.def_readonly("obj_model_", &Class::obj_model_);
    cls.def("setModel", &Class::setModel, "model"_a);
}

void defineRecognitionHypothesis(py::module &m) {
    using Class = pcl::recognition::Hypothesis;
    py::class_<Class, pcl::recognition::HypothesisBase, boost::shared_ptr<Class>> cls(m, "Hypothesis");
    cls.def(py::init<ModelLibrary::Model*>(), "obj_model"_a=NULL);
    cls.def_readwrite("match_confidence_", &Class::match_confidence_);
    cls.def_readwrite("explained_pixels_", &Class::explained_pixels_);
    cls.def_readwrite("linear_id_", &Class::linear_id_);
    // Operators not implemented (operator=);
    cls.def("computeBounds", &Class::computeBounds, "bounds"_a);
    cls.def("computeCenterOfMass", &Class::computeCenterOfMass, "center_of_mass"_a);
    cls.def("setLinearId", &Class::setLinearId, "id"_a);
    cls.def("getLinearId", &Class::getLinearId);
}

void defineRecognitionHypothesisFunctions(py::module &m) {
}

void defineRecognitionHypothesisClasses(py::module &sub_module) {
    defineRecognitionHypothesisBase(sub_module);
    defineRecognitionHypothesis(sub_module);
}