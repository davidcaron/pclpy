
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


void defineFeaturesClasses(py::module &m) {
    py::module m_features = m.def_submodule("features", "Submodule features");
    defineFeaturesOrganizedEdgeDetectionClasses(m_features);
    defineFeaturesPfhToolsClasses(m_features);
    defineFeaturesStatisticalMultiscaleInterestRegionExtractionClasses(m_features);
    defineFeaturesFeatureClasses(m_features);
    defineFeatures3dscClasses(m_features);
    defineFeaturesBoardClasses(m_features);
}