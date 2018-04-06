
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/orr_graph.h>

using namespace pcl::recognition;


template<class NodeData>
void defineRecognitionORRGraph(py::module &m, std::string const & suffix) {
    using Class = recognition::ORRGraph<NodeData>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_readonly("nodes_", &Class::nodes_);
    cls.def("clear", &Class::clear);
    cls.def("resize", &Class::resize);
    cls.def("compute_maximal_on_off_partition", &Class::computeMaximalOnOffPartition);
    cls.def("insert_undirected_edge", &Class::insertUndirectedEdge);
    cls.def("insert_directed_edge", &Class::insertDirectedEdge);
    cls.def("delete_undirected_edge", &Class::deleteUndirectedEdge);
    cls.def("delete_directed_edge", &Class::deleteDirectedEdge);
        
}

void defineRecognitionOrrGraphClasses(py::module &sub_module) {
}