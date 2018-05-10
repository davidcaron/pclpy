
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "common/angles.hpp"
#include "common/bivariate_polynomial.hpp"
#include "common/centroid.hpp"
#include "common/colors.hpp"
#include "common/common.hpp"
#include "common/copy_point.hpp"
#include "common/distances.hpp"
#include "common/feature_histogram.hpp"
#include "common/file_io.hpp"
#include "common/intersections.hpp"
#include "common/io.hpp"
#include "common/norms.hpp"
#include "common/piecewise_linear_function.hpp"
#include "common/point_tests.hpp"
#include "common/poses_from_matches.hpp"
#include "common/projection_matrix.hpp"
#include "common/time.hpp"
#include "common/transformation_from_correspondences.hpp"
#include "common/transforms.hpp"


void defineCommonClasses(py::module &m) {
    py::module m_common = m.def_submodule("common", "Submodule common");
    defineCommonAnglesClasses(m_common);
    defineCommonBivariatePolynomialClasses(m_common);
    defineCommonCentroidClasses(m_common);
    defineCommonColorsClasses(m_common);
    defineCommonCommonClasses(m_common);
    defineCommonCopyPointClasses(m_common);
    defineCommonDistancesClasses(m_common);
    defineCommonFeatureHistogramClasses(m_common);
    defineCommonFileIoClasses(m_common);
    defineCommonIntersectionsClasses(m_common);
    defineCommonIoClasses(m_common);
    defineCommonNormsClasses(m_common);
    defineCommonPiecewiseLinearFunctionClasses(m_common);
    defineCommonPointTestsClasses(m_common);
    defineCommonPosesFromMatchesClasses(m_common);
    defineCommonProjectionMatrixClasses(m_common);
    defineCommonTimeClasses(m_common);
    defineCommonTransformationFromCorrespondencesClasses(m_common);
    defineCommonTransformsClasses(m_common);
}