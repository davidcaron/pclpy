
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "kdtree/io.hpp"
#include "kdtree/kdtree.hpp"


void defineKdtreeClasses(py::module &m) {
    py::module m_kdtree = m.def_submodule("kdtree", "Submodule kdtree");
    defineKdtreeIoClasses(m_kdtree);
    defineKdtreeKdtreeClasses(m_kdtree);
}