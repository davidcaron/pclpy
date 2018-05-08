
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

void defineSurfaceAllocatorClasses(py::module &);
void defineSurfaceBinaryNodeClasses(py::module &);
void defineSurfaceBoostClasses(py::module &);
void defineSurfaceBsplineDataClasses(py::module &);
void defineSurfaceEigenClasses(py::module &);
void defineSurfaceFactorClasses(py::module &);
void defineSurfaceFunctionDataClasses(py::module &);
void defineSurfaceGeometryClasses(py::module &);
void defineSurfaceHashClasses(py::module &);
void defineSurfaceMarchingCubesPoissonClasses(py::module &);
void defineSurfaceMatClasses(py::module &);
void defineSurfaceNurbsDataClasses(py::module &);
void defineSurfaceNurbsToolsClasses(py::module &);
void defineSurfaceOctreePoissonClasses(py::module &);
void defineSurfacePolynomialClasses(py::module &);
void defineSurfacePpolynomialClasses(py::module &);
void defineSurfaceProcessingClasses(py::module &);
void defineSurfaceBilateralUpsamplingClasses(py::module &);
void defineSurfaceEarClippingClasses(py::module &);
void defineSurfaceMlsClasses(py::module &);
void defineSurfaceQhullClasses(py::module &);
void defineSurfaceReconstructionClasses(py::module &);
void defineSurfaceConvexHullClasses(py::module &);
void defineSurfaceConcaveHullClasses(py::module &);
void defineSurfaceGp3Classes(py::module &);
void defineSurfaceGridProjectionClasses(py::module &);
void defineSurfaceMarchingCubesClasses(py::module &);
void defineSurfaceMarchingCubesHoppeClasses(py::module &);
void defineSurfaceMarchingCubesRbfClasses(py::module &);
void defineSurfaceOrganizedFastMeshClasses(py::module &);
void defineSurfacePoissonClasses(py::module &);
void defineSurfaceSimplificationRemoveUnusedVerticesClasses(py::module &);
void defineSurfaceSparseMatClasses(py::module &);
void defineSurfaceNurbsSolveClasses(py::module &);
void defineSurfaceFittingCurve2dClasses(py::module &);
void defineSurfaceFittingCurve2dApdmClasses(py::module &);
void defineSurfaceFittingCurve2dAsdmClasses(py::module &);
void defineSurfaceFittingCurve2dAtdmClasses(py::module &);
void defineSurfaceFittingCurve2dPdmClasses(py::module &);
void defineSurfaceFittingCurve2dSdmClasses(py::module &);
void defineSurfaceFittingCurve2dTdmClasses(py::module &);
void defineSurfaceFittingCurvePdmClasses(py::module &);
void defineSurfaceFittingCylinderPdmClasses(py::module &);
void defineSurfaceFittingSpherePdmClasses(py::module &);
void defineSurfaceFittingSurfaceImClasses(py::module &);
void defineSurfaceFittingSurfacePdmClasses(py::module &);
void defineSurfaceClosingBoundaryClasses(py::module &);
void defineSurfaceFittingSurfaceTdmClasses(py::module &);
void defineSurfaceGlobalOptimizationPdmClasses(py::module &);
void defineSurfaceGlobalOptimizationTdmClasses(py::module &);
void defineSurfaceSequentialFitterClasses(py::module &);
void defineSurfaceSparseMatrixClasses(py::module &);
void defineSurfaceSurfelSmoothingClasses(py::module &);
void defineSurfaceTextureMappingClasses(py::module &);
void defineSurfaceTriangulationClasses(py::module &);
void defineSurfaceVectorClasses(py::module &);
void defineSurfaceVtkClasses(py::module &);
void defineSurfaceVtkMeshQuadricDecimationClasses(py::module &);
void defineSurfaceVtkMeshSmoothingLaplacianClasses(py::module &);
void defineSurfaceVtkMeshSmoothingWindowedSincClasses(py::module &);
void defineSurfaceVtkMeshSubdivisionClasses(py::module &);
void defineSurfaceVtkUtilsClasses(py::module &);


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