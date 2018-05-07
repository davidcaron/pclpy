
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/common/transformation_from_correspondences.h>



void defineCommonTransformationFromCorrespondences(py::module &m) {
    using Class = pcl::TransformationFromCorrespondences;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "TransformationFromCorrespondences");
    cls.def(py::init<>());
    cls.def("reset", &Class::reset);
    cls.def("add", &Class::add, "point"_a, "corresponding_point"_a, "weight"_a=1.0);
    cls.def("getAccumulatedWeight", &Class::getAccumulatedWeight);
    cls.def("getNoOfSamples", &Class::getNoOfSamples);
    cls.def("getTransformation", &Class::getTransformation);
}

void defineCommonTransformationFromCorrespondencesFunctions(py::module &m) {
}

void defineCommonTransformationFromCorrespondencesClasses(py::module &sub_module) {
    defineCommonTransformationFromCorrespondences(sub_module);
}