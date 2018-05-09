
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"
#pragma warning(disable : 4800)
#include <pcl/recognition/ransac_based/orr_octree.h>

using namespace pcl::recognition;


void defineRecognitionORROctree(py::module &m) {
    using Class = pcl::recognition::ORROctree;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudN = Class::PointCloudN;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "ORROctree");
    cls.def(py::init<>());
    cls.def("clear", &Class::clear);
    cls.def("build", py::overload_cast<const PointCloudIn &, float, const PointCloudN *, float> (&Class::build), "points"_a, "voxel_size"_a, "normals"_a=NULL, "enlarge_bounds"_a=0.00001f);
    cls.def("build", py::overload_cast<const float *, float> (&Class::build), "bounds"_a, "voxel_size"_a);
    cls.def("deleteBranch", &Class::deleteBranch, "node"_a);
    cls.def("insertNeighbors", &Class::insertNeighbors, "node"_a);
    cls.def("getFullLeavesIntersectedBySphere", &Class::getFullLeavesIntersectedBySphere, "p"_a, "radius"_a, "out"_a);
    cls.def("getRandomFullLeafOnSphere", &Class::getRandomFullLeafOnSphere, "p"_a, "radius"_a);
    cls.def("getLeaf", py::overload_cast<int, int, int> (&Class::getLeaf), "i"_a, "j"_a, "k"_a);
    cls.def("getLeaf", py::overload_cast<float, float, float> (&Class::getLeaf), "x"_a, "y"_a, "z"_a);
    cls.def("getFullLeaves", py::overload_cast<> (&Class::getFullLeaves));
    cls.def("getFullLeaves", py::overload_cast<> (&Class::getFullLeaves, py::const_));
    cls.def("getFullLeavesPoints", &Class::getFullLeavesPoints, "out"_a);
    cls.def("getNormalsOfFullLeaves", &Class::getNormalsOfFullLeaves, "out"_a);
    cls.def("getRoot", &Class::getRoot);
    cls.def("getBounds", py::overload_cast<> (&Class::getBounds, py::const_));
    cls.def("getBounds", py::overload_cast<float[6]> (&Class::getBounds, py::const_), "b"_a);
    cls.def("getVoxelSize", &Class::getVoxelSize);
}

void defineRecognitionOrrOctreeFunctions(py::module &m) {
}

void defineRecognitionOrrOctreeClasses(py::module &sub_module) {
    defineRecognitionORROctree(sub_module);
}