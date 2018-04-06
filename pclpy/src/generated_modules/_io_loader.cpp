
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"
#include "io/boost.hpp"
#include "io/auto_io.hpp"
#include "io/debayer.hpp"
#include "io/eigen.hpp"
#include "io/file_grabber.hpp"
#include "io/file_io.hpp"
#include "io/ascii_io.hpp"
#include "io/grabber.hpp"
#include "io/hdl_grabber.hpp"
#include "io/ifs_io.hpp"
#include "io/image_grabber.hpp"
#include "io/image_metadata_wrapper.hpp"
#include "io/image.hpp"
#include "io/image_rgb24.hpp"
#include "io/image_yuv422.hpp"
#include "io/io.hpp"
#include "io/lzf.hpp"
#include "io/lzf_image_io.hpp"
#include "io/pcd_grabber.hpp"
#include "io/pcd_io.hpp"
#include "io/ply_io.hpp"
#include "io/point_cloud_image_extractors.hpp"
#include "io/png_io.hpp"
#include "io/robot_eye_grabber.hpp"
#include "io/tar.hpp"
#include "io/vlp_grabber.hpp"


void defineIoClasses(py::module &m) {
    py::module m_io = m.def_submodule("io", "Submodule io");
    defineIoBoostClasses(m_io);
    defineIoAutoIoClasses(m_io);
    defineIoDebayerClasses(m_io);
    defineIoEigenClasses(m_io);
    defineIoFileGrabberClasses(m_io);
    defineIoFileIoClasses(m_io);
    defineIoAsciiIoClasses(m_io);
    defineIoGrabberClasses(m_io);
    defineIoHdlGrabberClasses(m_io);
    defineIoIfsIoClasses(m_io);
    defineIoImageGrabberClasses(m_io);
    defineIoImageMetadataWrapperClasses(m_io);
    defineIoImageClasses(m_io);
    defineIoImageRgb24Classes(m_io);
    defineIoImageYuv422Classes(m_io);
    defineIoIoClasses(m_io);
    defineIoLzfClasses(m_io);
    defineIoLzfImageIoClasses(m_io);
    defineIoPcdGrabberClasses(m_io);
    defineIoPcdIoClasses(m_io);
    defineIoPlyIoClasses(m_io);
    defineIoPointCloudImageExtractorsClasses(m_io);
    defineIoPngIoClasses(m_io);
    defineIoRobotEyeGrabberClasses(m_io);
    defineIoTarClasses(m_io);
    defineIoVlpGrabberClasses(m_io);
}