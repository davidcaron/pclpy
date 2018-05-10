
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "tracking/coherence.hpp"
#include "tracking/distance_coherence.hpp"
#include "tracking/hsv_color_coherence.hpp"


void defineTrackingClasses0(py::module &m) {
    py::module m_tracking = m.def_submodule("tracking", "Submodule tracking");
    defineTrackingCoherenceClasses(m_tracking);
    defineTrackingDistanceCoherenceClasses(m_tracking);
    defineTrackingHsvColorCoherenceClasses(m_tracking);
}