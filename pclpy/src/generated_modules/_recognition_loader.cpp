
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineRecognitionAuxiliaryClasses(py::module &);
void defineRecognitionBoostClasses(py::module &);
void defineRecognitionBvhClasses(py::module &);
void defineRecognitionCorrespondenceGroupingClasses(py::module &);
void defineRecognitionDistanceMapClasses(py::module &);
void defineRecognitionGeometricConsistencyClasses(py::module &);
void defineRecognitionHough3dClasses(py::module &);
void defineRecognitionMaskMapClasses(py::module &);
void defineRecognitionOcclusionReasoningClasses(py::module &);
void defineRecognitionOrrGraphClasses(py::module &);
void defineRecognitionOrrOctreeClasses(py::module &);
void defineRecognitionOrrOctreeZprojectionClasses(py::module &);
void defineRecognitionPointTypesClasses(py::module &);
void defineRecognitionQuantizedMapClasses(py::module &);
void defineRecognitionRegionXyClasses(py::module &);
void defineRecognitionDenseQuantizedMultiModTemplateClasses(py::module &);
void defineRecognitionDotModalityClasses(py::module &);
void defineRecognitionColorGradientDotModalityClasses(py::module &);
void defineRecognitionDotmodClasses(py::module &);
void defineRecognitionSimpleOctreeClasses(py::module &);
void defineRecognitionSparseQuantizedMultiModTemplateClasses(py::module &);
void defineRecognitionQuantizableModalityClasses(py::module &);
void defineRecognitionColorGradientModalityClasses(py::module &);
void defineRecognitionColorModalityClasses(py::module &);
void defineRecognitionLinemodClasses(py::module &);
void defineRecognitionVoxelStructureClasses(py::module &);
void defineRecognitionModelLibraryClasses(py::module &);
void defineRecognitionHypothesisClasses(py::module &);
void defineRecognitionRigidTransformSpaceClasses(py::module &);
void defineRecognitionCrhAlignmentClasses(py::module &);
void defineRecognitionImplicitShapeModelClasses(py::module &);
void defineRecognitionSurfaceNormalModalityClasses(py::module &);
void defineRecognitionLineRgbdClasses(py::module &);
void defineRecognitionHypothesesVerificationClasses(py::module &);
void defineRecognitionGreedyVerificationClasses(py::module &);
void defineRecognitionHvPapazovClasses(py::module &);


void defineRecognitionClasses(py::module &m) {
    py::module m_recognition = m.def_submodule("recognition", "Submodule recognition");
    defineRecognitionAuxiliaryClasses(m_recognition);
    defineRecognitionBoostClasses(m_recognition);
    defineRecognitionBvhClasses(m_recognition);
    defineRecognitionCorrespondenceGroupingClasses(m_recognition);
    defineRecognitionDistanceMapClasses(m_recognition);
    defineRecognitionGeometricConsistencyClasses(m_recognition);
    defineRecognitionHough3dClasses(m_recognition);
    defineRecognitionMaskMapClasses(m_recognition);
    defineRecognitionOcclusionReasoningClasses(m_recognition);
    defineRecognitionOrrGraphClasses(m_recognition);
    defineRecognitionOrrOctreeClasses(m_recognition);
    defineRecognitionOrrOctreeZprojectionClasses(m_recognition);
    defineRecognitionPointTypesClasses(m_recognition);
    defineRecognitionQuantizedMapClasses(m_recognition);
    defineRecognitionRegionXyClasses(m_recognition);
    defineRecognitionDenseQuantizedMultiModTemplateClasses(m_recognition);
    defineRecognitionDotModalityClasses(m_recognition);
    defineRecognitionColorGradientDotModalityClasses(m_recognition);
    defineRecognitionDotmodClasses(m_recognition);
    defineRecognitionSimpleOctreeClasses(m_recognition);
    defineRecognitionSparseQuantizedMultiModTemplateClasses(m_recognition);
    defineRecognitionQuantizableModalityClasses(m_recognition);
    defineRecognitionColorGradientModalityClasses(m_recognition);
    defineRecognitionColorModalityClasses(m_recognition);
    defineRecognitionLinemodClasses(m_recognition);
    defineRecognitionVoxelStructureClasses(m_recognition);
    defineRecognitionModelLibraryClasses(m_recognition);
    defineRecognitionHypothesisClasses(m_recognition);
    defineRecognitionRigidTransformSpaceClasses(m_recognition);
    defineRecognitionCrhAlignmentClasses(m_recognition);
    defineRecognitionImplicitShapeModelClasses(m_recognition);
    defineRecognitionSurfaceNormalModalityClasses(m_recognition);
    defineRecognitionLineRgbdClasses(m_recognition);
    defineRecognitionHypothesesVerificationClasses(m_recognition);
    defineRecognitionGreedyVerificationClasses(m_recognition);
    defineRecognitionHvPapazovClasses(m_recognition);
}