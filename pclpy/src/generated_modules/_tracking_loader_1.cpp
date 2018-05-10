
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "tracking/approx_nearest_pair_point_cloud_coherence.hpp"
#include "tracking/normal_coherence.hpp"
#include "tracking/tracker.hpp"
#include "tracking/particle_filter.hpp"


void defineTrackingClasses(py::module &m) {
    py::module m_tracking = m.def_submodule("tracking", "Submodule tracking");
    defineTrackingApproxNearestPairPointCloudCoherenceClasses(m_tracking);
    defineTrackingNormalCoherenceClasses(m_tracking);
    defineTrackingTrackerClasses(m_tracking);
    defineTrackingParticleFilterClasses(m_tracking);
}