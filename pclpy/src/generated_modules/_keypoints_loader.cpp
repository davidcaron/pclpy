
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineKeypointsUniformSamplingClasses(py::module &);
void defineKeypointsKeypointClasses(py::module &);
void defineKeypointsAgast2dClasses(py::module &);
void defineKeypointsBrisk2dClasses(py::module &);
void defineKeypointsHarris2dClasses(py::module &);
void defineKeypointsHarris3dClasses(py::module &);
void defineKeypointsHarris6dClasses(py::module &);
void defineKeypointsIss3dClasses(py::module &);
void defineKeypointsSiftKeypointClasses(py::module &);
void defineKeypointsSusanClasses(py::module &);


void defineKeypointsClasses(py::module &m) {
    py::module m_keypoints = m.def_submodule("keypoints", "Submodule keypoints");
    defineKeypointsUniformSamplingClasses(m_keypoints);
    defineKeypointsKeypointClasses(m_keypoints);
    defineKeypointsAgast2dClasses(m_keypoints);
    defineKeypointsBrisk2dClasses(m_keypoints);
    defineKeypointsHarris2dClasses(m_keypoints);
    defineKeypointsHarris3dClasses(m_keypoints);
    defineKeypointsHarris6dClasses(m_keypoints);
    defineKeypointsIss3dClasses(m_keypoints);
    defineKeypointsSiftKeypointClasses(m_keypoints);
    defineKeypointsSusanClasses(m_keypoints);
}