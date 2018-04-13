#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

namespace py = pybind11;
using namespace pybind11::literals;

static py::class_<Eigen::Quaternionf, boost::shared_ptr<Eigen::Quaternionf>>
                    defineQuaternion(py::module & m) {
    using Class = Eigen::Quaternionf;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Quaternionf");
    cls.def(py::init<>());
    // todo: make this usable...
    // For now, this enables compilation because Quaternion is not in pybind11/eigen.h
    return cls;
}

void defineEigenClasses(py::module &m) {
    defineQuaternion(m);
}