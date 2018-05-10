
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "features/organized_edge_detection.hpp"
#include "features/pfh_tools.hpp"
#include "features/statistical_multiscale_interest_region_extraction.hpp"
#include "features/feature.hpp"
#include "features/3dsc.hpp"
#include "features/board.hpp"
#include "features/boundary.hpp"
#include "features/cppf.hpp"
#include "features/crh.hpp"
#include "features/don.hpp"
#include "features/esf.hpp"
#include "features/fpfh.hpp"
#include "features/fpfh_omp.hpp"
#include "features/gfpfh.hpp"
#include "features/integral_image_normal.hpp"
#include "features/intensity_gradient.hpp"
#include "features/intensity_spin.hpp"
#include "features/linear_least_squares_normal.hpp"
#include "features/moment_invariants.hpp"
#include "features/moment_of_inertia_estimation.hpp"
#include "features/multiscale_feature_persistence.hpp"
#include "features/normal_3d.hpp"
#include "features/normal_3d_omp.hpp"
#include "features/normal_based_signature.hpp"
#include "features/pfh.hpp"
#include "features/pfhrgb.hpp"
#include "features/ppf.hpp"
#include "features/ppfrgb.hpp"
#include "features/principal_curvatures.hpp"
#include "features/rift.hpp"
#include "features/rops_estimation.hpp"
#include "features/rsd.hpp"
#include "features/grsd.hpp"
#include "features/shot.hpp"
#include "features/shot_lrf.hpp"
#include "features/shot_lrf_omp.hpp"
#include "features/shot_omp.hpp"
#include "features/spin_image.hpp"
#include "features/usc.hpp"
#include "features/vfh.hpp"
#include "features/cvfh.hpp"
#include "features/our_cvfh.hpp"


void defineFeaturesClasses(py::module &m) {
    py::module m_features = m.def_submodule("features", "Submodule features");
    defineFeaturesOrganizedEdgeDetectionClasses(m_features);
    defineFeaturesPfhToolsClasses(m_features);
    defineFeaturesStatisticalMultiscaleInterestRegionExtractionClasses(m_features);
    defineFeaturesFeatureClasses(m_features);
    defineFeatures3dscClasses(m_features);
    defineFeaturesBoardClasses(m_features);
    defineFeaturesBoundaryClasses(m_features);
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