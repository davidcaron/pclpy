
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "io/debayer.hpp"
#include "io/file_io.hpp"
#include "io/ascii_io.hpp"
#include "io/grabber.hpp"
#include "io/hdl_grabber.hpp"
#include "io/ifs_io.hpp"
#include "io/image_depth.hpp"
#include "io/image_grabber.hpp"
#include "io/image_metadata_wrapper.hpp"
#include "io/image.hpp"


void defineIoClasses(py::module &m) {
    py::module m_io = m.def_submodule("io", "Submodule io");
    defineIoDebayerClasses(m_io);
    defineIoFileIoClasses(m_io);
    defineIoAsciiIoClasses(m_io);
    defineIoGrabberClasses(m_io);
    defineIoHdlGrabberClasses(m_io);
    defineIoIfsIoClasses(m_io);
    defineIoImageDepthClasses(m_io);
    defineIoImageGrabberClasses(m_io);
    defineIoImageMetadataWrapperClasses(m_io);
    defineIoImageClasses(m_io);
}