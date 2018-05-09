
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineStereoDisparityMapConverterClasses(py::module &);
void defineStereoDigitalElevationMapClasses(py::module &);
void defineStereoStereoMatchingClasses(py::module &);
void defineStereoStereoGrabberClasses(py::module &);


void defineStereoClasses(py::module &m) {
    py::module m_stereo = m.def_submodule("stereo", "Submodule stereo");
    defineStereoDisparityMapConverterClasses(m_stereo);
    defineStereoDigitalElevationMapClasses(m_stereo);
    defineStereoStereoMatchingClasses(m_stereo);
    defineStereoStereoGrabberClasses(m_stereo);
}