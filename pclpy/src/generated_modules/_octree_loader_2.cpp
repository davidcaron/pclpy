
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "octree/octree_pointcloud_voxelcentroid.hpp"
#include "octree/octree_search.hpp"


void defineOctreeClasses2(py::module &m) {
    py::module m_octree = m.def_submodule("octree", "Submodule octree");
    defineOctreeOctreePointcloudVoxelcentroidClasses(m_octree);
    defineOctreeOctreeSearchClasses(m_octree);
}