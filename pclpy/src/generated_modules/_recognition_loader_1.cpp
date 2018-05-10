
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "recognition/orr_octree.hpp"
#include "recognition/orr_octree_zprojection.hpp"
#include "recognition/point_types.hpp"


void defineRecognitionClasses1(py::module &m) {
    py::module m_recognition = m.def_submodule("recognition", "Submodule recognition");
    defineRecognitionOrrOctreeClasses(m_recognition);
    defineRecognitionOrrOctreeZprojectionClasses(m_recognition);
    defineRecognitionPointTypesClasses(m_recognition);
}