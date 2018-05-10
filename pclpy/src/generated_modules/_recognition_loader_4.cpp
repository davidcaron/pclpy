
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "recognition/rigid_transform_space.hpp"
#include "recognition/implicit_shape_model.hpp"
#include "recognition/surface_normal_modality.hpp"
#include "recognition/line_rgbd.hpp"


void defineRecognitionClasses4(py::module &m) {
    py::module m_recognition = m.def_submodule("recognition", "Submodule recognition");
    defineRecognitionRigidTransformSpaceClasses(m_recognition);
    defineRecognitionImplicitShapeModelClasses(m_recognition);
    defineRecognitionSurfaceNormalModalityClasses(m_recognition);
    defineRecognitionLineRgbdClasses(m_recognition);
}