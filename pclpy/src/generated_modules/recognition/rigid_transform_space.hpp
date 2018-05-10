
#include <pcl/recognition/rigid_transform_space.h>

using namespace pcl::recognition;


void defineRecognitionRigidTransformSpace(py::module &m) {
    using Class = pcl::recognition::RigidTransformSpace;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RigidTransformSpace");
    cls.def(py::init<>());
    cls.def("build", py::overload_cast<const float *, float, float> (&Class::build), "pos_bounds"_a, "translation_cell_size"_a, "rotation_cell_size"_a);
    cls.def("clear", &Class::clear);
    cls.def("addRigidTransform", &Class::addRigidTransform, "model"_a, "position"_a, "rigid_transform"_a);
    cls.def("getRotationSpaces", py::overload_cast<> (&Class::getRotationSpaces));
    cls.def("getRotationSpaces", py::overload_cast<> (&Class::getRotationSpaces, py::const_));
    cls.def("getNumberOfOccupiedRotationSpaces", &Class::getNumberOfOccupiedRotationSpaces);
}

void defineRecognitionRotationSpace(py::module &m) {
    using Class = pcl::recognition::RotationSpace;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RotationSpace");
    cls.def(py::init<float>(), "discretization"_a);
    cls.def("addRigidTransform", &Class::addRigidTransform, "model"_a, "axis_angle"_a, "translation"_a);
    cls.def("setCenter", &Class::setCenter, "c"_a);
    cls.def("getCenter", &Class::getCenter);
    cls.def("getTransformWithMostVotes", &Class::getTransformWithMostVotes, "model"_a, "rigid_transform"_a);
}

void defineRecognitionRotationSpaceCell(py::module &m) {
    using Class = pcl::recognition::RotationSpaceCell;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RotationSpaceCell");
    cls.def(py::init<>());
    cls.def("addRigidTransform", &Class::addRigidTransform, "model"_a, "axis_angle"_a, "translation"_a);
    cls.def("getEntries", &Class::getEntries);
    cls.def("getEntry", &Class::getEntry, "model"_a);
}

void defineRecognitionRotationSpaceCellCreator(py::module &m) {
    using Class = pcl::recognition::RotationSpaceCellCreator;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RotationSpaceCellCreator");
    cls.def(py::init<>());
    cls.def("create", &Class::create, ""_a);
}

void defineRecognitionRotationSpaceCreator(py::module &m) {
    using Class = pcl::recognition::RotationSpaceCreator;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RotationSpaceCreator");
    cls.def(py::init<>());
    cls.def("create", &Class::create, "leaf"_a);
    cls.def("reset", &Class::reset);
    cls.def("setDiscretization", &Class::setDiscretization, "value"_a);
    cls.def("getNumberOfRotationSpaces", &Class::getNumberOfRotationSpaces);
    cls.def("getRotationSpaces", py::overload_cast<> (&Class::getRotationSpaces, py::const_));
    cls.def("getRotationSpaces", py::overload_cast<> (&Class::getRotationSpaces));
}

void defineRecognitionRigidTransformSpaceFunctions(py::module &m) {
}

void defineRecognitionRigidTransformSpaceClasses(py::module &sub_module) {
    defineRecognitionRigidTransformSpace(sub_module);
    defineRecognitionRotationSpace(sub_module);
    defineRecognitionRotationSpaceCell(sub_module);
    defineRecognitionRotationSpaceCellCreator(sub_module);
    defineRecognitionRotationSpaceCreator(sub_module);
}