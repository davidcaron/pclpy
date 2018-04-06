
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/simple_octree.h>

using namespace pcl::recognition;


template<typename NodeData, typename NodeDataCreator, typename Scalar = float>
void defineRecognitionSimpleOctree(py::module &m, std::string const & suffix) {
    using Class = recognition::SimpleOctree<NodeData, NodeDataCreator, Scalar>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("build", py::overload_cast<const Scalar*, Scalar, NodeDataCreator*> (&Class::build));
    cls.def("clear", &Class::clear);
    cls.def("create_leaf", &Class::createLeaf);
    cls.def("get_full_leaf", py::overload_cast<int, int, int> (&Class::getFullLeaf));
    cls.def("get_full_leaf", py::overload_cast<Scalar, Scalar, Scalar> (&Class::getFullLeaf));
    cls.def("get_full_leaves", py::overload_cast<> (&Class::getFullLeaves));
    cls.def("get_full_leaves", py::overload_cast<> (&Class::getFullLeaves, py::const_));
    cls.def("get_bounds", py::overload_cast<> (&Class::getBounds, py::const_));
    cls.def("get_bounds", py::overload_cast<Scalar[6]> (&Class::getBounds, py::const_));
        
}

void defineRecognitionSimpleOctreeClasses(py::module &sub_module) {
}