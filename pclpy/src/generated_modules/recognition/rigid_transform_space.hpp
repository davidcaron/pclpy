
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/rigid_transform_space.h>

using namespace pcl::recognition;


void defineRecognitionRigidTransformSpace(py::module &m) {
    using Class = recognition::RigidTransformSpace;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RigidTransformSpace");
    cls.def(py::init<>());
    cls.def("build", py::overload_cast<const float *, float, float> (&Class::build));
    cls.def("clear", &Class::clear);
    cls.def("add_rigid_transform", &Class::addRigidTransform);
    cls.def("get_rotation_spaces", py::overload_cast<> (&Class::getRotationSpaces));
    cls.def("get_rotation_spaces", py::overload_cast<> (&Class::getRotationSpaces, py::const_));
}

void defineRecognitionRotationSpace(py::module &m) {
    using Class = recognition::RotationSpace;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RotationSpace");
    cls.def(py::init<float>(), "discretization"_a);
    cls.def_property("center", &Class::getCenter, &Class::setCenter);
    cls.def("add_rigid_transform", &Class::addRigidTransform);
}

void defineRecognitionRotationSpaceCell(py::module &m) {
    using Class = recognition::RotationSpaceCell;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RotationSpaceCell");
    cls.def(py::init<>());
    cls.def("add_rigid_transform", &Class::addRigidTransform);
}

void defineRecognitionRotationSpaceCellCreator(py::module &m) {
    using Class = recognition::RotationSpaceCellCreator;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RotationSpaceCellCreator");
    cls.def(py::init<>());
    cls.def("create", &Class::create);
}

void defineRecognitionRotationSpaceCreator(py::module &m) {
    using Class = recognition::RotationSpaceCreator;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RotationSpaceCreator");
    cls.def(py::init<>());
    cls.def("set_discretization", &Class::setDiscretization);
    cls.def("create", &Class::create);
    cls.def("reset", &Class::reset);
    cls.def("get_rotation_spaces", py::overload_cast<> (&Class::getRotationSpaces, py::const_));
    cls.def("get_rotation_spaces", py::overload_cast<> (&Class::getRotationSpaces));
}

void defineRecognitionRigidTransformSpaceClasses(py::module &sub_module) {
    defineRecognitionRigidTransformSpace(sub_module);
    defineRecognitionRotationSpace(sub_module);
    defineRecognitionRotationSpaceCell(sub_module);
    defineRecognitionRotationSpaceCellCreator(sub_module);
    defineRecognitionRotationSpaceCreator(sub_module);
}