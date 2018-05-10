
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "common/distances.hpp"
#include "common/feature_histogram.hpp"
#include "common/file_io.hpp"
#include "common/intersections.hpp"
#include "common/io.hpp"
#include "common/norms.hpp"


void defineCommonClasses(py::module &m) {
    py::module m_common = m.def_submodule("common", "Submodule common");
    defineCommonDistancesClasses(m_common);
    defineCommonFeatureHistogramClasses(m_common);
    defineCommonFileIoClasses(m_common);
    defineCommonIntersectionsClasses(m_common);
    defineCommonIoClasses(m_common);
    defineCommonNormsClasses(m_common);
}