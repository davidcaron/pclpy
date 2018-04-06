
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/octree/octree_container.h>

using namespace pcl::octree;


void defineOctreeOctreeContainerBase(py::module &m) {
    using Class = octree::OctreeContainerBase;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "OctreeContainerBase");
    // Operators not implemented (operator==);
    // Operators not implemented (operator!=);
    cls.def("reset", &Class::reset);
    cls.def("add_point_index", &Class::addPointIndex);
}

void defineOctreeOctreeContainerEmpty(py::module &m) {
    using Class = octree::OctreeContainerEmpty;
    py::class_<Class, OctreeContainerBase, boost::shared_ptr<Class>> cls(m, "OctreeContainerEmpty");
    cls.def(py::init<>());
    cls.def("deep_copy", &Class::deepCopy);
    cls.def("reset", &Class::reset);
    cls.def("add_point_index", &Class::addPointIndex);
}

void defineOctreeOctreeContainerPointIndex(py::module &m) {
    using Class = octree::OctreeContainerPointIndex;
    py::class_<Class, OctreeContainerBase, boost::shared_ptr<Class>> cls(m, "OctreeContainerPointIndex");
    cls.def(py::init<>());
    cls.def("deep_copy", &Class::deepCopy);
    // Operators not implemented (operator==);
    cls.def("add_point_index", &Class::addPointIndex);
    cls.def("reset", &Class::reset);
}

void defineOctreeOctreeContainerPointIndices(py::module &m) {
    using Class = octree::OctreeContainerPointIndices;
    py::class_<Class, OctreeContainerBase, boost::shared_ptr<Class>> cls(m, "OctreeContainerPointIndices");
    cls.def(py::init<>());
    cls.def("deep_copy", &Class::deepCopy);
    // Operators not implemented (operator==);
    cls.def("add_point_index", &Class::addPointIndex);
    cls.def("reset", &Class::reset);
}

void defineOctreeOctreeContainerClasses(py::module &sub_module) {
    defineOctreeOctreeContainerBase(sub_module);
    defineOctreeOctreeContainerEmpty(sub_module);
    defineOctreeOctreeContainerPointIndex(sub_module);
    defineOctreeOctreeContainerPointIndices(sub_module);
}