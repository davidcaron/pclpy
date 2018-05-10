
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "common/poses_from_matches.hpp"
#include "common/projection_matrix.hpp"


void defineCommonClasses7(py::module &m) {
    py::module m_common = m.def_submodule("common", "Submodule common");
    defineCommonPosesFromMatchesClasses(m_common);
    defineCommonProjectionMatrixClasses(m_common);
}