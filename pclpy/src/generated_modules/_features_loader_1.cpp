
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "features/crh.hpp"
#include "features/don.hpp"
#include "features/esf.hpp"
#include "features/fpfh.hpp"
#include "features/fpfh_omp.hpp"
#include "features/gfpfh.hpp"
#include "features/integral_image_normal.hpp"
#include "features/intensity_gradient.hpp"


void defineFeaturesClasses1(py::module &m) {
    py::module m_features = m.def_submodule("features", "Submodule features");
    defineFeaturesCrhClasses(m_features);
    defineFeaturesDonClasses(m_features);
    defineFeaturesEsfClasses(m_features);
    defineFeaturesFpfhClasses(m_features);
    defineFeaturesFpfhOmpClasses(m_features);
    defineFeaturesGfpfhClasses(m_features);
    defineFeaturesIntegralImageNormalClasses(m_features);
    defineFeaturesIntensityGradientClasses(m_features);
}