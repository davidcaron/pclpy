
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/orr_octree.h>

using namespace pcl::recognition;


void defineRecognitionORROctree(py::module &m) {
    using Class = recognition::ORROctree;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudN = Class::PointCloudN;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "ORROctree");
    cls.def(py::init<>());
    cls.def("build", py::overload_cast<const PointCloudIn &, float, const PointCloudN *, float> (&Class::build));
    cls.def("build", py::overload_cast<const float *, float> (&Class::build));
    cls.def("clear", &Class::clear);
    cls.def("delete_branch", &Class::deleteBranch);
    cls.def("insert_neighbors", &Class::insertNeighbors);
    cls.def("get_leaf", py::overload_cast<int, int, int> (&Class::getLeaf));
    cls.def("get_leaf", py::overload_cast<float, float, float> (&Class::getLeaf));
    cls.def("get_full_leaves", py::overload_cast<> (&Class::getFullLeaves));
    cls.def("get_full_leaves", py::overload_cast<> (&Class::getFullLeaves, py::const_));
    cls.def("get_bounds", py::overload_cast<> (&Class::getBounds, py::const_));
    cls.def("get_bounds", py::overload_cast<float[6]> (&Class::getBounds, py::const_));
}

void defineRecognitionOrrOctreeClasses(py::module &sub_module) {
    defineRecognitionORROctree(sub_module);
}