
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

#include <pcl/recognition/ransac_based/bvh.h>

using namespace pcl::recognition;


template<class UserData>
void defineRecognitionBVH(py::module &m, std::string const & suffix) {
    using Class = pcl::recognition::BVH<UserData>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("build", py::overload_cast<std::vector<BoundedObject *> &> (&Class::build), "objects"_a);
    cls.def("clear", &Class::clear);
    cls.def("intersect", &Class::intersect, "box"_a, "intersected_objects"_a);
    cls.def("getInputObjects", &Class::getInputObjects);
        
}

void defineRecognitionBvhFunctions(py::module &m) {
}

void defineRecognitionBvhClasses(py::module &sub_module) {
}