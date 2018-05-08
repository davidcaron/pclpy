
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

#include <pcl/recognition/ransac_based/orr_graph.h>

using namespace pcl::recognition;


template<class NodeData>
void defineRecognitionORRGraph(py::module &m, std::string const & suffix) {
    using Class = pcl::recognition::ORRGraph<NodeData>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_readwrite("nodes_", &Class::nodes_);
    cls.def("clear", &Class::clear);
    cls.def("resize", &Class::resize, "n"_a);
    cls.def("computeMaximalOnOffPartition", &Class::computeMaximalOnOffPartition, "on_nodes"_a, "off_nodes"_a);
    cls.def("insertUndirectedEdge", &Class::insertUndirectedEdge, "id1"_a, "id2"_a);
    cls.def("insertDirectedEdge", &Class::insertDirectedEdge, "id1"_a, "id2"_a);
    cls.def("deleteUndirectedEdge", &Class::deleteUndirectedEdge, "id1"_a, "id2"_a);
    cls.def("deleteDirectedEdge", &Class::deleteDirectedEdge, "id1"_a, "id2"_a);
    cls.def("getNodes", &Class::getNodes);
        
}

void defineRecognitionOrrGraphFunctions(py::module &m) {
}

void defineRecognitionOrrGraphClasses(py::module &sub_module) {
}