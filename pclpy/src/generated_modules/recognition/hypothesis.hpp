
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/hypothesis.h>

using namespace pcl::recognition;


void defineRecognitionHypothesisBase(py::module &m) {
    using Class = recognition::HypothesisBase;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "HypothesisBase");
    cls.def(py::init<ModelLibrary::Model*>(), "obj_model"_a);
    cls.def(py::init<ModelLibrary::Model*, float*>(), "obj_model"_a, "rigid_transform"_a);
    cls.def("set_model", &Class::setModel);
    cls.def_readonly("rigid_transform_", &Class::rigid_transform_);
    cls.def_readonly("obj_model_", &Class::obj_model_);
}

void defineRecognitionHypothesis(py::module &m) {
    using Class = recognition::Hypothesis;
    py::class_<Class, HypothesisBase, boost::shared_ptr<Class>> cls(m, "Hypothesis");
    cls.def(py::init<ModelLibrary::Model*>(), "obj_model"_a=NULL);
    cls.def_property("linear_id", &Class::getLinearId, &Class::setLinearId);
    cls.def_readonly("match_confidence_", &Class::match_confidence_);
    cls.def_readonly("explained_pixels_", &Class::explained_pixels_);
    cls.def_readonly("linear_id_", &Class::linear_id_);
    // Operators not implemented (operator=);
    cls.def("compute_bounds", &Class::computeBounds);
    cls.def("compute_center_of_mass", &Class::computeCenterOfMass);
}

void defineRecognitionHypothesisClasses(py::module &sub_module) {
    defineRecognitionHypothesisBase(sub_module);
    defineRecognitionHypothesis(sub_module);
}