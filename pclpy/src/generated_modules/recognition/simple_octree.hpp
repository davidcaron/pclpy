
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/recognition/simple_octree.h>

using namespace pcl::recognition;


template<typename NodeData, typename NodeDataCreator, typename Scalar = float>
void defineRecognitionSimpleOctree(py::module &m, std::string const & suffix) {
    using Class = pcl::recognition::SimpleOctree<NodeData, NodeDataCreator, Scalar>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("clear", &Class::clear);
    cls.def("build", py::overload_cast<const Scalar*, Scalar, NodeDataCreator*> (&Class::build), "bounds"_a, "voxel_size"_a, "node_data_creator"_a);
    cls.def("createLeaf", &Class::createLeaf, "x"_a, "y"_a, "z"_a);
    cls.def("getFullLeaf", py::overload_cast<int, int, int> (&Class::getFullLeaf), "i"_a, "j"_a, "k"_a);
    cls.def("getFullLeaf", py::overload_cast<Scalar, Scalar, Scalar> (&Class::getFullLeaf), "x"_a, "y"_a, "z"_a);
    cls.def("getFullLeaves", py::overload_cast<> (&Class::getFullLeaves));
    cls.def("getFullLeaves", py::overload_cast<> (&Class::getFullLeaves, py::const_));
    cls.def("getRoot", &Class::getRoot);
    cls.def("getBounds", py::overload_cast<> (&Class::getBounds, py::const_));
    cls.def("getBounds", py::overload_cast<Scalar[6]> (&Class::getBounds, py::const_), "b"_a);
    cls.def("getVoxelSize", &Class::getVoxelSize);
        
}

void defineRecognitionSimpleOctreeFunctions(py::module &m) {
}

void defineRecognitionSimpleOctreeClasses(py::module &sub_module) {
}