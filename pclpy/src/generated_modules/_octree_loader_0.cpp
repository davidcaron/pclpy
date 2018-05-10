
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "octree/octree_container.hpp"
#include "octree/octree_key.hpp"
#include "octree/octree_nodes.hpp"
#include "octree/octree_iterator.hpp"
#include "octree/octree2buf_base.hpp"
#include "octree/octree_base.hpp"
#include "octree/octree_pointcloud.hpp"
#include "octree/octree_pointcloud_density.hpp"


void defineOctreeClasses0(py::module &m) {
    py::module m_octree = m.def_submodule("octree", "Submodule octree");
    defineOctreeOctreeContainerClasses(m_octree);
    defineOctreeOctreeKeyClasses(m_octree);
    defineOctreeOctreeNodesClasses(m_octree);
    defineOctreeOctreeIteratorClasses(m_octree);
    defineOctreeOctree2bufBaseClasses(m_octree);
    defineOctreeOctreeBaseClasses(m_octree);
    defineOctreeOctreePointcloudClasses(m_octree);
    defineOctreeOctreePointcloudDensityClasses(m_octree);
}