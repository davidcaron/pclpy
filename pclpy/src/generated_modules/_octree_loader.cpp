
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"
#include "octree/boost.hpp"
#include "octree/octree_container.hpp"
#include "octree/octree_key.hpp"
#include "octree/octree_nodes.hpp"
#include "octree/octree_iterator.hpp"
#include "octree/octree2buf_base.hpp"
#include "octree/octree_base.hpp"
#include "octree/octree_pointcloud.hpp"
#include "octree/octree_pointcloud_adjacency_container.hpp"
#include "octree/octree_pointcloud_adjacency.hpp"
#include "octree/octree_pointcloud_changedetector.hpp"
#include "octree/octree_pointcloud_density.hpp"
#include "octree/octree_pointcloud_occupancy.hpp"
#include "octree/octree_pointcloud_pointvector.hpp"
#include "octree/octree_pointcloud_singlepoint.hpp"
#include "octree/octree_pointcloud_voxelcentroid.hpp"
#include "octree/octree_search.hpp"
#include "octree/octree.hpp"
#include "octree/octree_impl.hpp"


void defineOctreeClasses(py::module &m) {
    py::module m_octree = m.def_submodule("octree", "Submodule octree");
    defineOctreeBoostClasses(m_octree);
    defineOctreeOctreeContainerClasses(m_octree);
    defineOctreeOctreeKeyClasses(m_octree);
    defineOctreeOctreeNodesClasses(m_octree);
    defineOctreeOctreeIteratorClasses(m_octree);
    defineOctreeOctree2bufBaseClasses(m_octree);
    defineOctreeOctreeBaseClasses(m_octree);
    defineOctreeOctreePointcloudClasses(m_octree);
    defineOctreeOctreePointcloudAdjacencyContainerClasses(m_octree);
    defineOctreeOctreePointcloudAdjacencyClasses(m_octree);
    defineOctreeOctreePointcloudChangedetectorClasses(m_octree);
    defineOctreeOctreePointcloudDensityClasses(m_octree);
    defineOctreeOctreePointcloudOccupancyClasses(m_octree);
    defineOctreeOctreePointcloudPointvectorClasses(m_octree);
    defineOctreeOctreePointcloudSinglepointClasses(m_octree);
    defineOctreeOctreePointcloudVoxelcentroidClasses(m_octree);
    defineOctreeOctreeSearchClasses(m_octree);
    defineOctreeOctreeClasses(m_octree);
    defineOctreeOctreeImplClasses(m_octree);
}