
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "sample_consensus/method_types.hpp"
#include "sample_consensus/model_types.hpp"
#include "sample_consensus/sac_model.hpp"
#include "sample_consensus/sac.hpp"
#include "sample_consensus/lmeds.hpp"


void defineSampleConsensusClasses0(py::module &m) {
    py::module m_sample_consensus = m.def_submodule("sample_consensus", "Submodule sample_consensus");
    defineSampleConsensusMethodTypesClasses(m_sample_consensus);
    defineSampleConsensusModelTypesClasses(m_sample_consensus);
    defineSampleConsensusSacModelClasses(m_sample_consensus);
    defineSampleConsensusSacClasses(m_sample_consensus);
    defineSampleConsensusLmedsClasses(m_sample_consensus);
}