
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

#include <pcl/recognition/hv/hypotheses_verification.h>



template<typename ModelT, typename SceneT>
void defineRecognitionHypothesisVerification(py::module &m, std::string const & suffix) {
    using Class = pcl::HypothesisVerification<ModelT, SceneT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("addCompleteModels", &Class::addCompleteModels, "complete_models"_a);
    cls.def("addNormalsClouds", &Class::addNormalsClouds, "complete_models"_a);
    cls.def("addModels", &Class::addModels, "models"_a, "occlusion_reasoning"_a=false);
    cls.def("verify", &Class::verify);
    cls.def("setResolution", &Class::setResolution, "r"_a);
    cls.def("setOcclusionThreshold", &Class::setOcclusionThreshold, "t"_a);
    cls.def("setInlierThreshold", &Class::setInlierThreshold, "r"_a);
    cls.def("setSceneCloud", &Class::setSceneCloud, "scene_cloud"_a);
    cls.def("setOcclusionCloud", &Class::setOcclusionCloud, "occ_cloud"_a);
    cls.def("getRequiresNormals", &Class::getRequiresNormals);
    cls.def("getMask", &Class::getMask, "mask"_a);
        
}

void defineRecognitionHypothesesVerificationFunctions(py::module &m) {
}

void defineRecognitionHypothesesVerificationClasses(py::module &sub_module) {
}