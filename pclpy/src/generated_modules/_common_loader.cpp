
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
#include "common/_kiss_fft_guts.hpp"
#include "common/angles.hpp"
#include "common/bivariate_polynomial.hpp"
#include "common/boost.hpp"
#include "common/centroid.hpp"
#include "common/colors.hpp"
#include "common/common.hpp"
#include "common/concatenate.hpp"
#include "common/copy_point.hpp"
#include "common/distances.hpp"
#include "common/feature_histogram.hpp"
#include "common/file_io.hpp"
#include "common/geometry.hpp"
#include "common/intensity.hpp"
#include "common/intersections.hpp"
#include "common/io.hpp"
#include "common/kiss_fft.hpp"
#include "common/kiss_fftr.hpp"
#include "common/norms.hpp"
#include "common/pca.hpp"
#include "common/piecewise_linear_function.hpp"
#include "common/point_operators.hpp"
#include "common/point_tests.hpp"
#include "common/polynomial_calculations.hpp"
#include "common/poses_from_matches.hpp"
#include "common/projection_matrix.hpp"
#include "common/random.hpp"
#include "common/generate.hpp"
#include "common/spring.hpp"
#include "common/time.hpp"
#include "common/common_headers.hpp"
#include "common/transformation_from_correspondences.hpp"
#include "common/transforms.hpp"
#include "common/utils.hpp"
#include "common/vector_average.hpp"


void defineCommonClasses(py::module &m) {
    py::module m_common = m.def_submodule("common", "Submodule common");
    defineCommon_kissFftGutsClasses(m_common);
    defineCommonAnglesClasses(m_common);
    defineCommonBivariatePolynomialClasses(m_common);
    defineCommonBoostClasses(m_common);
    defineCommonCentroidClasses(m_common);
    defineCommonColorsClasses(m_common);
    defineCommonCommonClasses(m_common);
    defineCommonConcatenateClasses(m_common);
    defineCommonCopyPointClasses(m_common);
    defineCommonDistancesClasses(m_common);
    defineCommonFeatureHistogramClasses(m_common);
    defineCommonFileIoClasses(m_common);
    defineCommonGeometryClasses(m_common);
    defineCommonIntensityClasses(m_common);
    defineCommonIntersectionsClasses(m_common);
    defineCommonIoClasses(m_common);
    defineCommonKissFftClasses(m_common);
    defineCommonKissFftrClasses(m_common);
    defineCommonNormsClasses(m_common);
    defineCommonPcaClasses(m_common);
    defineCommonPiecewiseLinearFunctionClasses(m_common);
    defineCommonPointOperatorsClasses(m_common);
    defineCommonPointTestsClasses(m_common);
    defineCommonPolynomialCalculationsClasses(m_common);
    defineCommonPosesFromMatchesClasses(m_common);
    defineCommonProjectionMatrixClasses(m_common);
    defineCommonRandomClasses(m_common);
    defineCommonGenerateClasses(m_common);
    defineCommonSpringClasses(m_common);
    defineCommonTimeClasses(m_common);
    defineCommonCommonHeadersClasses(m_common);
    defineCommonTransformationFromCorrespondencesClasses(m_common);
    defineCommonTransformsClasses(m_common);
    defineCommonUtilsClasses(m_common);
    defineCommonVectorAverageClasses(m_common);
}