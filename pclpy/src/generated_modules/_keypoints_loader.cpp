
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
#include "keypoints/uniform_sampling.hpp"
#include "keypoints/keypoint.hpp"
#include "keypoints/agast_2d.hpp"
#include "keypoints/brisk_2d.hpp"
#include "keypoints/harris_2d.hpp"
#include "keypoints/harris_3d.hpp"
#include "keypoints/harris_6d.hpp"
#include "keypoints/iss_3d.hpp"
#include "keypoints/sift_keypoint.hpp"
#include "keypoints/susan.hpp"


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