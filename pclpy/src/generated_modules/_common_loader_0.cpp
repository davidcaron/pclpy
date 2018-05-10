
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
}