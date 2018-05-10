
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "surface/reconstruction.hpp"
#include "surface/convex_hull.hpp"
#include "surface/concave_hull.hpp"
#include "surface/gp3.hpp"


void defineSurfaceClasses1(py::module &m) {
    py::module m_surface = m.def_submodule("surface", "Submodule surface");
    defineSurfaceReconstructionClasses(m_surface);
    defineSurfaceConvexHullClasses(m_surface);
    defineSurfaceConcaveHullClasses(m_surface);
    defineSurfaceGp3Classes(m_surface);
}