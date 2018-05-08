
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineSampleConsensusBoostClasses(py::module &);
void defineSampleConsensusEigenClasses(py::module &);
void defineSampleConsensusMethodTypesClasses(py::module &);
void defineSampleConsensusModelTypesClasses(py::module &);
void defineSampleConsensusSacModelClasses(py::module &);
void defineSampleConsensusSacClasses(py::module &);
void defineSampleConsensusLmedsClasses(py::module &);
void defineSampleConsensusMlesacClasses(py::module &);
void defineSampleConsensusMsacClasses(py::module &);
void defineSampleConsensusProsacClasses(py::module &);
void defineSampleConsensusRansacClasses(py::module &);
void defineSampleConsensusRmsacClasses(py::module &);
void defineSampleConsensusRransacClasses(py::module &);
void defineSampleConsensusSacModelCircleClasses(py::module &);
void defineSampleConsensusSacModelCircle3dClasses(py::module &);
void defineSampleConsensusSacModelConeClasses(py::module &);
void defineSampleConsensusSacModelCylinderClasses(py::module &);
void defineSampleConsensusSacModelLineClasses(py::module &);
void defineSampleConsensusSacModelParallelLineClasses(py::module &);
void defineSampleConsensusSacModelPlaneClasses(py::module &);
void defineSampleConsensusSacModelParallelPlaneClasses(py::module &);
void defineSampleConsensusSacModelPerpendicularPlaneClasses(py::module &);
void defineSampleConsensusSacModelNormalPlaneClasses(py::module &);
void defineSampleConsensusSacModelNormalParallelPlaneClasses(py::module &);
void defineSampleConsensusSacModelRegistrationClasses(py::module &);
void defineSampleConsensusSacModelRegistration2dClasses(py::module &);
void defineSampleConsensusSacModelSphereClasses(py::module &);
void defineSampleConsensusSacModelNormalSphereClasses(py::module &);
void defineSampleConsensusSacModelStickClasses(py::module &);


void defineSampleConsensusClasses(py::module &m) {
    py::module m_sample_consensus = m.def_submodule("sample_consensus", "Submodule sample_consensus");
    defineSampleConsensusBoostClasses(m_sample_consensus);
    defineSampleConsensusEigenClasses(m_sample_consensus);
    defineSampleConsensusMethodTypesClasses(m_sample_consensus);
    defineSampleConsensusModelTypesClasses(m_sample_consensus);
    defineSampleConsensusSacModelClasses(m_sample_consensus);
    defineSampleConsensusSacClasses(m_sample_consensus);
    defineSampleConsensusLmedsClasses(m_sample_consensus);
    defineSampleConsensusMlesacClasses(m_sample_consensus);
    defineSampleConsensusMsacClasses(m_sample_consensus);
    defineSampleConsensusProsacClasses(m_sample_consensus);
    defineSampleConsensusRansacClasses(m_sample_consensus);
    defineSampleConsensusRmsacClasses(m_sample_consensus);
    defineSampleConsensusRransacClasses(m_sample_consensus);
    defineSampleConsensusSacModelCircleClasses(m_sample_consensus);
    defineSampleConsensusSacModelCircle3dClasses(m_sample_consensus);
    defineSampleConsensusSacModelConeClasses(m_sample_consensus);
    defineSampleConsensusSacModelCylinderClasses(m_sample_consensus);
    defineSampleConsensusSacModelLineClasses(m_sample_consensus);
    defineSampleConsensusSacModelParallelLineClasses(m_sample_consensus);
    defineSampleConsensusSacModelPlaneClasses(m_sample_consensus);
    defineSampleConsensusSacModelParallelPlaneClasses(m_sample_consensus);
    defineSampleConsensusSacModelPerpendicularPlaneClasses(m_sample_consensus);
    defineSampleConsensusSacModelNormalPlaneClasses(m_sample_consensus);
    defineSampleConsensusSacModelNormalParallelPlaneClasses(m_sample_consensus);
    defineSampleConsensusSacModelRegistrationClasses(m_sample_consensus);
    defineSampleConsensusSacModelRegistration2dClasses(m_sample_consensus);
    defineSampleConsensusSacModelSphereClasses(m_sample_consensus);
    defineSampleConsensusSacModelNormalSphereClasses(m_sample_consensus);
    defineSampleConsensusSacModelStickClasses(m_sample_consensus);
}