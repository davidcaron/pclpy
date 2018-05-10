
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "octree/octree_base.hpp"
#include "octree/octree_pointcloud.hpp"
#include "octree/octree_pointcloud_density.hpp"
#include "octree/octree_pointcloud_voxelcentroid.hpp"
#include "octree/octree_search.hpp"


void defineOctreeClasses1(py::module &m) {
    py::module m_octree = m.def_submodule("octree", "Submodule octree");
    defineOctreeOctreeBaseClasses(m_octree);
    defineOctreeOctreePointcloudClasses(m_octree);
    defineOctreeOctreePointcloudDensityClasses(m_octree);
    defineOctreeOctreePointcloudVoxelcentroidClasses(m_octree);
    defineOctreeOctreeSearchClasses(m_octree);
}