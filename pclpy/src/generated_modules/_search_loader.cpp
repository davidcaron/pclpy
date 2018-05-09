
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineSearchSearchClasses(py::module &);
void defineSearchBruteForceClasses(py::module &);
void defineSearchFlannSearchClasses(py::module &);
void defineSearchKdtreeClasses(py::module &);
void defineSearchOctreeClasses(py::module &);
void defineSearchOrganizedClasses(py::module &);
void defineSearchPclSearchClasses(py::module &);


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