
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "sample_consensus/sac_model_normal_plane.hpp"
#include "sample_consensus/sac_model_normal_parallel_plane.hpp"
#include "sample_consensus/sac_model_registration.hpp"
#include "sample_consensus/sac_model_sphere.hpp"


void defineSampleConsensusClasses(py::module &m) {
    py::module m_sample_consensus = m.def_submodule("sample_consensus", "Submodule sample_consensus");
    defineSampleConsensusSacModelNormalPlaneClasses(m_sample_consensus);
    defineSampleConsensusSacModelNormalParallelPlaneClasses(m_sample_consensus);
    defineSampleConsensusSacModelRegistrationClasses(m_sample_consensus);
    defineSampleConsensusSacModelSphereClasses(m_sample_consensus);
}