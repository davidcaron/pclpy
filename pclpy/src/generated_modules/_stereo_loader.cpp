
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
#include "stereo/disparity_map_converter.hpp"
#include "stereo/digital_elevation_map.hpp"
#include "stereo/stereo_matching.hpp"
#include "stereo/stereo_grabber.hpp"


void defineStereoClasses(py::module &m) {
    py::module m_stereo = m.def_submodule("stereo", "Submodule stereo");
    defineStereoDisparityMapConverterClasses(m_stereo);
    defineStereoDigitalElevationMapClasses(m_stereo);
    defineStereoStereoMatchingClasses(m_stereo);
    defineStereoStereoGrabberClasses(m_stereo);
}