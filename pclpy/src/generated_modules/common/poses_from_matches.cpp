
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

#include <pcl/common/poses_from_matches.h>



void defineCommonPosesFromMatches(py::module &m) {
    using Class = pcl::PosesFromMatches;
    using PoseEstimatesVector = Class::PoseEstimatesVector;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PosesFromMatches");
    cls.def(py::init<>());
    cls.def("estimatePosesUsing1Correspondence", &Class::estimatePosesUsing1Correspondence, "correspondences"_a, "max_no_of_results"_a, "pose_estimates"_a);
    cls.def("estimatePosesUsing2Correspondences", &Class::estimatePosesUsing2Correspondences, "correspondences"_a, "max_no_of_tested_combinations"_a, "max_no_of_results"_a, "pose_estimates"_a);
    cls.def("estimatePosesUsing3Correspondences", &Class::estimatePosesUsing3Correspondences, "correspondences"_a, "max_no_of_tested_combinations"_a, "max_no_of_results"_a, "pose_estimates"_a);
    cls.def("getParameters", &Class::getParameters);
}

void defineCommonPosesFromMatchesFunctions(py::module &m) {
}

void defineCommonPosesFromMatchesClasses(py::module &sub_module) {
    defineCommonPosesFromMatches(sub_module);
}