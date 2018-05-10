
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "surface/ear_clipping.hpp"
#include "surface/mls.hpp"


void defineSurfaceClasses1(py::module &m) {
    py::module m_surface = m.def_submodule("surface", "Submodule surface");
    defineSurfaceEarClippingClasses(m_surface);
    defineSurfaceMlsClasses(m_surface);
}