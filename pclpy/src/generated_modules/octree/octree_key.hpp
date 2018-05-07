
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/octree/octree_key.h>

using namespace pcl::octree;


void defineOctreeOctreeKey(py::module &m) {
    using Class = pcl::octree::OctreeKey;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "OctreeKey");
    cls.def(py::init<>());
    cls.def(py::init<unsigned int, unsigned int, unsigned int>(), "keyX"_a, "keyY"_a, "keyZ"_a);
    cls.def_readonly_static("maxDepth", &Class::maxDepth);
    cls.def_readonly("key_", &Class::key_);
    // Operators not implemented (operator==);
    // Operators not implemented (operator<=);
    // Operators not implemented (operator>=);
    cls.def("pushBranch", &Class::pushBranch, "childIndex"_a);
    cls.def("popBranch", &Class::popBranch);
    cls.def("getChildIdxWithDepthMask", &Class::getChildIdxWithDepthMask, "depthMask"_a);
}

void defineOctreeOctreeKeyFunctions(py::module &m) {
}

void defineOctreeOctreeKeyClasses(py::module &sub_module) {
    defineOctreeOctreeKey(sub_module);
}