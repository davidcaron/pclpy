
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "search/organized.hpp"


void defineSearchClasses1(py::module &m) {
    py::module m_search = m.def_submodule("search", "Submodule search");
    defineSearchOrganizedClasses(m_search);
}