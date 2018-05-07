
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/octree/octree_container.h>

using namespace pcl::octree;


void defineOctreeOctreeContainerBase(py::module &m) {
    using Class = pcl::octree::OctreeContainerBase;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "OctreeContainerBase");
    // Operators not implemented (operator==);
    // Operators not implemented (operator!=);
    cls.def("reset", &Class::reset);
    cls.def("addPointIndex", &Class::addPointIndex, "&"_a);
    cls.def("getSize", &Class::getSize);
    cls.def("getPointIndex", &Class::getPointIndex, "&"_a);
    cls.def("getPointIndices", &Class::getPointIndices, "&"_a);
}

void defineOctreeOctreeContainerEmpty(py::module &m) {
    using Class = pcl::octree::OctreeContainerEmpty;
    py::class_<Class, pcl::octree::OctreeContainerBase, boost::shared_ptr<Class>> cls(m, "OctreeContainerEmpty");
    cls.def(py::init<>());
    cls.def("deepCopy", &Class::deepCopy);
    cls.def("reset", &Class::reset);
    cls.def("addPointIndex", &Class::addPointIndex, ""_a);
    cls.def("getSize", &Class::getSize);
    cls.def("getPointIndex", &Class::getPointIndex);
    cls.def("getPointIndices", &Class::getPointIndices, "&"_a);
}

void defineOctreeOctreeContainerPointIndex(py::module &m) {
    using Class = pcl::octree::OctreeContainerPointIndex;
    py::class_<Class, pcl::octree::OctreeContainerBase, boost::shared_ptr<Class>> cls(m, "OctreeContainerPointIndex");
    cls.def(py::init<>());
    cls.def("deepCopy", &Class::deepCopy);
    // Operators not implemented (operator==);
    cls.def("addPointIndex", &Class::addPointIndex, "data_arg"_a);
    cls.def("reset", &Class::reset);
    cls.def("getPointIndex", &Class::getPointIndex);
    cls.def("getPointIndices", &Class::getPointIndices, "data_vector_arg"_a);
    cls.def("getSize", &Class::getSize);
}

void defineOctreeOctreeContainerPointIndices(py::module &m) {
    using Class = pcl::octree::OctreeContainerPointIndices;
    py::class_<Class, pcl::octree::OctreeContainerBase, boost::shared_ptr<Class>> cls(m, "OctreeContainerPointIndices");
    cls.def(py::init<>());
    cls.def("deepCopy", &Class::deepCopy);
    // Operators not implemented (operator==);
    cls.def("addPointIndex", &Class::addPointIndex, "data_arg"_a);
    cls.def("reset", &Class::reset);
    cls.def("getPointIndex", &Class::getPointIndex);
    cls.def("getPointIndices", &Class::getPointIndices, "data_vector_arg"_a);
    cls.def("getPointIndicesVector", &Class::getPointIndicesVector);
    cls.def("getSize", &Class::getSize);
}

void defineOctreeOctreeContainerFunctions(py::module &m) {
}

void defineOctreeOctreeContainerClasses(py::module &sub_module) {
    defineOctreeOctreeContainerBase(sub_module);
    defineOctreeOctreeContainerEmpty(sub_module);
    defineOctreeOctreeContainerPointIndex(sub_module);
    defineOctreeOctreeContainerPointIndices(sub_module);
}