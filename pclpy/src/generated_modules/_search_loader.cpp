
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
#include "search/search.hpp"
#include "search/brute_force.hpp"
#include "search/flann_search.hpp"
#include "search/kdtree.hpp"
#include "search/octree.hpp"
#include "search/organized.hpp"
#include "search/pcl_search.hpp"


void defineSearchClasses(py::module &m) {
    py::module m_search = m.def_submodule("search", "Submodule search");
    defineSearchSearchClasses(m_search);
    defineSearchBruteForceClasses(m_search);
    defineSearchFlannSearchClasses(m_search);
    defineSearchKdtreeClasses(m_search);
    defineSearchOctreeClasses(m_search);
    defineSearchOrganizedClasses(m_search);
    defineSearchPclSearchClasses(m_search);
}