
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "sample_consensus/ransac.hpp"
#include "sample_consensus/rmsac.hpp"
#include "sample_consensus/rransac.hpp"
#include "sample_consensus/sac_model_circle.hpp"
#include "sample_consensus/sac_model_circle3d.hpp"
#include "sample_consensus/sac_model_cone.hpp"
#include "sample_consensus/sac_model_cylinder.hpp"
#include "sample_consensus/sac_model_line.hpp"


void defineSampleConsensusClasses1(py::module &m) {
    py::module m_sample_consensus = m.def_submodule("sample_consensus", "Submodule sample_consensus");
    defineSampleConsensusRansacClasses(m_sample_consensus);
    defineSampleConsensusRmsacClasses(m_sample_consensus);
    defineSampleConsensusRransacClasses(m_sample_consensus);
    defineSampleConsensusSacModelCircleClasses(m_sample_consensus);
    defineSampleConsensusSacModelCircle3dClasses(m_sample_consensus);
    defineSampleConsensusSacModelConeClasses(m_sample_consensus);
    defineSampleConsensusSacModelCylinderClasses(m_sample_consensus);
    defineSampleConsensusSacModelLineClasses(m_sample_consensus);
}