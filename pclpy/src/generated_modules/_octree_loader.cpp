
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineOctreeBoostClasses(py::module &);
void defineOctreeOctreeContainerClasses(py::module &);
void defineOctreeOctreeKeyClasses(py::module &);
void defineOctreeOctreeNodesClasses(py::module &);
void defineOctreeOctreeIteratorClasses(py::module &);
void defineOctreeOctree2bufBaseClasses(py::module &);
void defineOctreeOctreeBaseClasses(py::module &);
void defineOctreeOctreePointcloudClasses(py::module &);
void defineOctreeOctreePointcloudAdjacencyContainerClasses(py::module &);
void defineOctreeOctreePointcloudAdjacencyClasses(py::module &);
void defineOctreeOctreePointcloudChangedetectorClasses(py::module &);
void defineOctreeOctreePointcloudDensityClasses(py::module &);
void defineOctreeOctreePointcloudOccupancyClasses(py::module &);
void defineOctreeOctreePointcloudPointvectorClasses(py::module &);
void defineOctreeOctreePointcloudSinglepointClasses(py::module &);
void defineOctreeOctreePointcloudVoxelcentroidClasses(py::module &);
void defineOctreeOctreeSearchClasses(py::module &);
void defineOctreeOctreeClasses(py::module &);
void defineOctreeOctreeImplClasses(py::module &);


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