
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

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


void defineFeaturesClasses(py::module &m) {
    py::module m_features = m.def_submodule("features", "Submodule features");
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
}