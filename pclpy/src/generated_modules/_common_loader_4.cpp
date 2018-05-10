
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "common/time.hpp"
#include "common/transformation_from_correspondences.hpp"
#include "common/transforms.hpp"


void defineCommonClasses(py::module &m) {
    py::module m_common = m.def_submodule("common", "Submodule common");
    defineCommonTimeClasses(m_common);
    defineCommonTransformationFromCorrespondencesClasses(m_common);
    defineCommonTransformsClasses(m_common);
}