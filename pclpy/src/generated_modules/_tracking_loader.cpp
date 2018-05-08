
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

void defineTrackingBoostClasses(py::module &);
void defineTrackingCoherenceClasses(py::module &);
void defineTrackingDistanceCoherenceClasses(py::module &);
void defineTrackingHsvColorCoherenceClasses(py::module &);
void defineTrackingNearestPairPointCloudCoherenceClasses(py::module &);
void defineTrackingApproxNearestPairPointCloudCoherenceClasses(py::module &);
void defineTrackingNormalCoherenceClasses(py::module &);
void defineTrackingTrackingClasses(py::module &);
void defineTrackingTrackerClasses(py::module &);
void defineTrackingParticleFilterClasses(py::module &);
void defineTrackingKldAdaptiveParticleFilterClasses(py::module &);
void defineTrackingKldAdaptiveParticleFilterOmpClasses(py::module &);
void defineTrackingParticleFilterOmpClasses(py::module &);
void defineTrackingPyramidalKltClasses(py::module &);


void defineTrackingClasses(py::module &m) {
    py::module m_tracking = m.def_submodule("tracking", "Submodule tracking");
    defineTrackingBoostClasses(m_tracking);
    defineTrackingCoherenceClasses(m_tracking);
    defineTrackingDistanceCoherenceClasses(m_tracking);
    defineTrackingHsvColorCoherenceClasses(m_tracking);
    defineTrackingNearestPairPointCloudCoherenceClasses(m_tracking);
    defineTrackingApproxNearestPairPointCloudCoherenceClasses(m_tracking);
    defineTrackingNormalCoherenceClasses(m_tracking);
    defineTrackingTrackingClasses(m_tracking);
    defineTrackingTrackerClasses(m_tracking);
    defineTrackingParticleFilterClasses(m_tracking);
    defineTrackingKldAdaptiveParticleFilterClasses(m_tracking);
    defineTrackingKldAdaptiveParticleFilterOmpClasses(m_tracking);
    defineTrackingParticleFilterOmpClasses(m_tracking);
    defineTrackingPyramidalKltClasses(m_tracking);
}