
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "features/shot_omp.hpp"
#include "features/spin_image.hpp"


void defineFeaturesClasses18(py::module &m) {
    py::module m_features = m.def_submodule("features", "Submodule features");
    defineFeaturesShotOmpClasses(m_features);
    defineFeaturesSpinImageClasses(m_features);
}