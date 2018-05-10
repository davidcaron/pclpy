
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "recognition/quantizable_modality.hpp"
#include "recognition/linemod.hpp"
#include "recognition/model_library.hpp"


void defineRecognitionClasses4(py::module &m) {
    py::module m_recognition = m.def_submodule("recognition", "Submodule recognition");
    defineRecognitionQuantizableModalityClasses(m_recognition);
    defineRecognitionLinemodClasses(m_recognition);
    defineRecognitionModelLibraryClasses(m_recognition);
}