
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/transformation_from_correspondences.h>



void defineCommonTransformationFromCorrespondences(py::module &m) {
    using Class = TransformationFromCorrespondences;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "TransformationFromCorrespondences");
    cls.def(py::init<>());
    cls.def("reset", &Class::reset);
    cls.def("add", &Class::add);
}

void defineCommonTransformationFromCorrespondencesClasses(py::module &sub_module) {
    defineCommonTransformationFromCorrespondences(sub_module);
}