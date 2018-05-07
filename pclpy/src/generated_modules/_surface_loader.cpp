
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
#include "surface/allocator.hpp"
#include "surface/binary_node.hpp"
#include "surface/boost.hpp"
#include "surface/bspline_data.hpp"
#include "surface/eigen.hpp"
#include "surface/factor.hpp"
#include "surface/function_data.hpp"
#include "surface/geometry.hpp"
#include "surface/hash.hpp"
#include "surface/marching_cubes_poisson.hpp"
#include "surface/mat.hpp"
#include "surface/nurbs_data.hpp"
#include "surface/nurbs_tools.hpp"
#include "surface/octree_poisson.hpp"
#include "surface/polynomial.hpp"
#include "surface/ppolynomial.hpp"
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
#include "surface/sparse_mat.hpp"
#include "surface/nurbs_solve.hpp"
#include "surface/fitting_curve_2d.hpp"
#include "surface/fitting_curve_2d_apdm.hpp"
#include "surface/fitting_curve_2d_asdm.hpp"
#include "surface/fitting_curve_2d_atdm.hpp"
#include "surface/fitting_curve_2d_pdm.hpp"
#include "surface/fitting_curve_2d_sdm.hpp"
#include "surface/fitting_curve_2d_tdm.hpp"
#include "surface/fitting_curve_pdm.hpp"
#include "surface/fitting_cylinder_pdm.hpp"
#include "surface/fitting_sphere_pdm.hpp"
#include "surface/fitting_surface_im.hpp"
#include "surface/fitting_surface_pdm.hpp"
#include "surface/closing_boundary.hpp"
#include "surface/fitting_surface_tdm.hpp"
#include "surface/global_optimization_pdm.hpp"
#include "surface/global_optimization_tdm.hpp"
#include "surface/sequential_fitter.hpp"
#include "surface/sparse_matrix.hpp"
#include "surface/surfel_smoothing.hpp"
#include "surface/texture_mapping.hpp"
#include "surface/triangulation.hpp"
#include "surface/vector.hpp"
#include "surface/vtk.hpp"
#include "surface/vtk_mesh_quadric_decimation.hpp"
#include "surface/vtk_mesh_smoothing_laplacian.hpp"
#include "surface/vtk_mesh_smoothing_windowed_sinc.hpp"
#include "surface/vtk_mesh_subdivision.hpp"
#include "surface/vtk_utils.hpp"


void defineSurfaceClasses(py::module &m) {
    py::module m_surface = m.def_submodule("surface", "Submodule surface");
    defineSurfaceAllocatorClasses(m_surface);
    defineSurfaceBinaryNodeClasses(m_surface);
    defineSurfaceBoostClasses(m_surface);
    defineSurfaceBsplineDataClasses(m_surface);
    defineSurfaceEigenClasses(m_surface);
    defineSurfaceFactorClasses(m_surface);
    defineSurfaceFunctionDataClasses(m_surface);
    defineSurfaceGeometryClasses(m_surface);
    defineSurfaceHashClasses(m_surface);
    defineSurfaceMarchingCubesPoissonClasses(m_surface);
    defineSurfaceMatClasses(m_surface);
    defineSurfaceNurbsDataClasses(m_surface);
    defineSurfaceNurbsToolsClasses(m_surface);
    defineSurfaceOctreePoissonClasses(m_surface);
    defineSurfacePolynomialClasses(m_surface);
    defineSurfacePpolynomialClasses(m_surface);
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
    defineSurfaceSparseMatClasses(m_surface);
    defineSurfaceNurbsSolveClasses(m_surface);
    defineSurfaceFittingCurve2dClasses(m_surface);
    defineSurfaceFittingCurve2dApdmClasses(m_surface);
    defineSurfaceFittingCurve2dAsdmClasses(m_surface);
    defineSurfaceFittingCurve2dAtdmClasses(m_surface);
    defineSurfaceFittingCurve2dPdmClasses(m_surface);
    defineSurfaceFittingCurve2dSdmClasses(m_surface);
    defineSurfaceFittingCurve2dTdmClasses(m_surface);
    defineSurfaceFittingCurvePdmClasses(m_surface);
    defineSurfaceFittingCylinderPdmClasses(m_surface);
    defineSurfaceFittingSpherePdmClasses(m_surface);
    defineSurfaceFittingSurfaceImClasses(m_surface);
    defineSurfaceFittingSurfacePdmClasses(m_surface);
    defineSurfaceClosingBoundaryClasses(m_surface);
    defineSurfaceFittingSurfaceTdmClasses(m_surface);
    defineSurfaceGlobalOptimizationPdmClasses(m_surface);
    defineSurfaceGlobalOptimizationTdmClasses(m_surface);
    defineSurfaceSequentialFitterClasses(m_surface);
    defineSurfaceSparseMatrixClasses(m_surface);
    defineSurfaceSurfelSmoothingClasses(m_surface);
    defineSurfaceTextureMappingClasses(m_surface);
    defineSurfaceTriangulationClasses(m_surface);
    defineSurfaceVectorClasses(m_surface);
    defineSurfaceVtkClasses(m_surface);
    defineSurfaceVtkMeshQuadricDecimationClasses(m_surface);
    defineSurfaceVtkMeshSmoothingLaplacianClasses(m_surface);
    defineSurfaceVtkMeshSmoothingWindowedSincClasses(m_surface);
    defineSurfaceVtkMeshSubdivisionClasses(m_surface);
    defineSurfaceVtkUtilsClasses(m_surface);
}