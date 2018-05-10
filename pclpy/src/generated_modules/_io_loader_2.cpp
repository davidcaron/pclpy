
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "io/image_metadata_wrapper.hpp"
#include "io/image.hpp"
#include "io/image_ir.hpp"
#include "io/image_rgb24.hpp"


void defineIoClasses(py::module &m) {
    py::module m_io = m.def_submodule("io", "Submodule io");
    defineIoImageMetadataWrapperClasses(m_io);
    defineIoImageClasses(m_io);
    defineIoImageIrClasses(m_io);
    defineIoImageRgb24Classes(m_io);
}