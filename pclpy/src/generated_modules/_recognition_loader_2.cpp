
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "recognition/quantized_map.hpp"
#include "recognition/region_xy.hpp"
#include "recognition/dense_quantized_multi_mod_template.hpp"


void defineRecognitionClasses2(py::module &m) {
    py::module m_recognition = m.def_submodule("recognition", "Submodule recognition");
    defineRecognitionQuantizedMapClasses(m_recognition);
    defineRecognitionRegionXyClasses(m_recognition);
    defineRecognitionDenseQuantizedMultiModTemplateClasses(m_recognition);
}