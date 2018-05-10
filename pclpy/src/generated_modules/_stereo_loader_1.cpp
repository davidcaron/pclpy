
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "stereo/stereo_grabber.hpp"


void defineStereoClasses1(py::module &m) {
    py::module m_stereo = m.def_submodule("stereo", "Submodule stereo");
    defineStereoStereoGrabberClasses(m_stereo);
}