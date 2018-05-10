
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "visualization/keyboard_event.hpp"
#include "visualization/mouse_event.hpp"


void defineVisualizationClasses2(py::module &m) {
    py::module m_visualization = m.def_submodule("visualization", "Submodule visualization");
    defineVisualizationKeyboardEventClasses(m_visualization);
    defineVisualizationMouseEventClasses(m_visualization);
}