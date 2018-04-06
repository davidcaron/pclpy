
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
#include "surface/boost.hpp"
#include "surface/eigen.hpp"
#include "surface/processing.hpp"
#include "surface/bilateral_upsampling.hpp"
#include "surface/ear_clipping.hpp"
#include "surface/mls.hpp"
#include "surface/qhull.hpp"
#include "surface/reconstruction.hpp"
#include "surface/convex_hull.hpp"
#include "surface/concave_hull.hpp"
#include "surface/gp3.hpp"
#include "surface/grid_projection.hpp"
#include "surface/marching_cubes.hpp"
#include "surface/marching_cubes_hoppe.hpp"
#include "surface/marching_cubes_rbf.hpp"
#include "surface/organized_fast_mesh.hpp"
#include "surface/poisson.hpp"
#include "surface/simplification_remove_unused_vertices.hpp"
#include "surface/surfel_smoothing.hpp"
#include "surface/texture_mapping.hpp"


void defineSurfaceClasses(py::module &m) {
    py::module m_surface = m.def_submodule("surface", "Submodule surface");
    defineSurfaceBoostClasses(m_surface);
    defineSurfaceEigenClasses(m_surface);
    defineSurfaceProcessingClasses(m_surface);
    defineSurfaceBilateralUpsamplingClasses(m_surface);
    defineSurfaceEarClippingClasses(m_surface);
    defineSurfaceMlsClasses(m_surface);
    defineSurfaceQhullClasses(m_surface);
    defineSurfaceReconstructionClasses(m_surface);
    defineSurfaceConvexHullClasses(m_surface);
    defineSurfaceConcaveHullClasses(m_surface);
    defineSurfaceGp3Classes(m_surface);
    defineSurfaceGridProjectionClasses(m_surface);
    defineSurfaceMarchingCubesClasses(m_surface);
    defineSurfaceMarchingCubesHoppeClasses(m_surface);
    defineSurfaceMarchingCubesRbfClasses(m_surface);
    defineSurfaceOrganizedFastMeshClasses(m_surface);
    defineSurfacePoissonClasses(m_surface);
    defineSurfaceSimplificationRemoveUnusedVerticesClasses(m_surface);
    defineSurfaceSurfelSmoothingClasses(m_surface);
    defineSurfaceTextureMappingClasses(m_surface);
}