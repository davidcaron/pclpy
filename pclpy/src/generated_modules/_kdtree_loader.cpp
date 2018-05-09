
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineKdtreeFlannClasses(py::module &);
void defineKdtreeIoClasses(py::module &);
void defineKdtreeKdtreeClasses(py::module &);
void defineKdtreeKdtreeFlannClasses(py::module &);


void defineKdtreeClasses(py::module &m) {
    py::module m_kdtree = m.def_submodule("kdtree", "Submodule kdtree");
    defineKdtreeFlannClasses(m_kdtree);
    defineKdtreeIoClasses(m_kdtree);
    defineKdtreeKdtreeClasses(m_kdtree);
    defineKdtreeKdtreeFlannClasses(m_kdtree);
}