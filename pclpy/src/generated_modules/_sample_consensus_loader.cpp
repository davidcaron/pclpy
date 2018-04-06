
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"
#include "sample_consensus/boost.hpp"
#include "sample_consensus/eigen.hpp"
#include "sample_consensus/method_types.hpp"
#include "sample_consensus/model_types.hpp"
#include "sample_consensus/sac_model.hpp"
#include "sample_consensus/sac.hpp"
#include "sample_consensus/lmeds.hpp"
#include "sample_consensus/mlesac.hpp"
#include "sample_consensus/msac.hpp"
#include "sample_consensus/prosac.hpp"
#include "sample_consensus/ransac.hpp"
#include "sample_consensus/rmsac.hpp"
#include "sample_consensus/rransac.hpp"
#include "sample_consensus/sac_model_circle.hpp"
#include "sample_consensus/sac_model_circle3d.hpp"
#include "sample_consensus/sac_model_cone.hpp"
#include "sample_consensus/sac_model_cylinder.hpp"
#include "sample_consensus/sac_model_line.hpp"
#include "sample_consensus/sac_model_parallel_line.hpp"
#include "sample_consensus/sac_model_plane.hpp"
#include "sample_consensus/sac_model_parallel_plane.hpp"
#include "sample_consensus/sac_model_perpendicular_plane.hpp"
#include "sample_consensus/sac_model_normal_plane.hpp"
#include "sample_consensus/sac_model_normal_parallel_plane.hpp"
#include "sample_consensus/sac_model_registration.hpp"
#include "sample_consensus/sac_model_registration_2d.hpp"
#include "sample_consensus/sac_model_sphere.hpp"
#include "sample_consensus/sac_model_normal_sphere.hpp"
#include "sample_consensus/sac_model_stick.hpp"


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