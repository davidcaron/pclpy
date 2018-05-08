
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

void defineFeaturesBoostClasses(py::module &);
void defineFeaturesEigenClasses(py::module &);
void defineFeaturesIntegralImage2DClasses(py::module &);
void defineFeaturesOrganizedEdgeDetectionClasses(py::module &);
void defineFeaturesPfhToolsClasses(py::module &);
void defineFeaturesStatisticalMultiscaleInterestRegionExtractionClasses(py::module &);
void defineFeaturesFeatureClasses(py::module &);
void defineFeatures3dscClasses(py::module &);
void defineFeaturesBoardClasses(py::module &);
void defineFeaturesBoundaryClasses(py::module &);
void defineFeaturesBrisk2dClasses(py::module &);
void defineFeaturesCppfClasses(py::module &);
void defineFeaturesCrhClasses(py::module &);
void defineFeaturesDonClasses(py::module &);
void defineFeaturesEsfClasses(py::module &);
void defineFeaturesFpfhClasses(py::module &);
void defineFeaturesFpfhOmpClasses(py::module &);
void defineFeaturesGfpfhClasses(py::module &);
void defineFeaturesIntegralImageNormalClasses(py::module &);
void defineFeaturesIntensityGradientClasses(py::module &);
void defineFeaturesIntensitySpinClasses(py::module &);
void defineFeaturesLinearLeastSquaresNormalClasses(py::module &);
void defineFeaturesMomentInvariantsClasses(py::module &);
void defineFeaturesMomentOfInertiaEstimationClasses(py::module &);
void defineFeaturesMultiscaleFeaturePersistenceClasses(py::module &);
void defineFeaturesNormal3dClasses(py::module &);
void defineFeaturesFromMeshesClasses(py::module &);
void defineFeaturesNormal3dOmpClasses(py::module &);
void defineFeaturesNormalBasedSignatureClasses(py::module &);
void defineFeaturesPfhClasses(py::module &);
void defineFeaturesPfhrgbClasses(py::module &);
void defineFeaturesPpfClasses(py::module &);
void defineFeaturesPpfrgbClasses(py::module &);
void defineFeaturesPrincipalCurvaturesClasses(py::module &);
void defineFeaturesRiftClasses(py::module &);
void defineFeaturesRopsEstimationClasses(py::module &);
void defineFeaturesRsdClasses(py::module &);
void defineFeaturesGrsdClasses(py::module &);
void defineFeaturesShotClasses(py::module &);
void defineFeaturesShotLrfClasses(py::module &);
void defineFeaturesShotLrfOmpClasses(py::module &);
void defineFeaturesShotOmpClasses(py::module &);
void defineFeaturesSpinImageClasses(py::module &);
void defineFeaturesUscClasses(py::module &);
void defineFeaturesVfhClasses(py::module &);
void defineFeaturesCvfhClasses(py::module &);
void defineFeaturesOurCvfhClasses(py::module &);


void defineFeaturesClasses(py::module &m) {
    py::module m_features = m.def_submodule("features", "Submodule features");
    defineFeaturesBoostClasses(m_features);
    defineFeaturesEigenClasses(m_features);
    defineFeaturesIntegralImage2DClasses(m_features);
    defineFeaturesOrganizedEdgeDetectionClasses(m_features);
    defineFeaturesPfhToolsClasses(m_features);
    defineFeaturesStatisticalMultiscaleInterestRegionExtractionClasses(m_features);
    defineFeaturesFeatureClasses(m_features);
    defineFeatures3dscClasses(m_features);
    defineFeaturesBoardClasses(m_features);
    defineFeaturesBoundaryClasses(m_features);
    defineFeaturesBrisk2dClasses(m_features);
    defineFeaturesCppfClasses(m_features);
    defineFeaturesCrhClasses(m_features);
    defineFeaturesDonClasses(m_features);
    defineFeaturesEsfClasses(m_features);
    defineFeaturesFpfhClasses(m_features);
    defineFeaturesFpfhOmpClasses(m_features);
    defineFeaturesGfpfhClasses(m_features);
    defineFeaturesIntegralImageNormalClasses(m_features);
    defineFeaturesIntensityGradientClasses(m_features);
    defineFeaturesIntensitySpinClasses(m_features);
    defineFeaturesLinearLeastSquaresNormalClasses(m_features);
    defineFeaturesMomentInvariantsClasses(m_features);
    defineFeaturesMomentOfInertiaEstimationClasses(m_features);
    defineFeaturesMultiscaleFeaturePersistenceClasses(m_features);
    defineFeaturesNormal3dClasses(m_features);
    defineFeaturesFromMeshesClasses(m_features);
    defineFeaturesNormal3dOmpClasses(m_features);
    defineFeaturesNormalBasedSignatureClasses(m_features);
    defineFeaturesPfhClasses(m_features);
    defineFeaturesPfhrgbClasses(m_features);
    defineFeaturesPpfClasses(m_features);
    defineFeaturesPpfrgbClasses(m_features);
    defineFeaturesPrincipalCurvaturesClasses(m_features);
    defineFeaturesRiftClasses(m_features);
    defineFeaturesRopsEstimationClasses(m_features);
    defineFeaturesRsdClasses(m_features);
    defineFeaturesGrsdClasses(m_features);
    defineFeaturesShotClasses(m_features);
    defineFeaturesShotLrfClasses(m_features);
    defineFeaturesShotLrfOmpClasses(m_features);
    defineFeaturesShotOmpClasses(m_features);
    defineFeaturesSpinImageClasses(m_features);
    defineFeaturesUscClasses(m_features);
    defineFeaturesVfhClasses(m_features);
    defineFeaturesCvfhClasses(m_features);
    defineFeaturesOurCvfhClasses(m_features);
}