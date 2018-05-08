
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

void defineIoBoostClasses(py::module &);
void defineIoAutoIoClasses(py::module &);
void defineIoByteOrderClasses(py::module &);
void defineIoDebayerClasses(py::module &);
void defineIoEigenClasses(py::module &);
void defineIoFileGrabberClasses(py::module &);
void defineIoFileIoClasses(py::module &);
void defineIoAsciiIoClasses(py::module &);
void defineIoGrabberClasses(py::module &);
void defineIoHdlGrabberClasses(py::module &);
void defineIoIfsIoClasses(py::module &);
void defineIoImageDepthClasses(py::module &);
void defineIoImageGrabberClasses(py::module &);
void defineIoImageMetadataWrapperClasses(py::module &);
void defineIoImageClasses(py::module &);
void defineIoImageIrClasses(py::module &);
void defineIoImageRgb24Classes(py::module &);
void defineIoImageYuv422Classes(py::module &);
void defineIoIoClasses(py::module &);
void defineIoIoExceptionClasses(py::module &);
void defineIoIoOperatorsClasses(py::module &);
void defineIoLzfClasses(py::module &);
void defineIoLzfImageIoClasses(py::module &);
void defineIoObjIoClasses(py::module &);
void defineIoPcdGrabberClasses(py::module &);
void defineIoPcdIoClasses(py::module &);
void defineIoPlyClasses(py::module &);
void defineIoPlyParserClasses(py::module &);
void defineIoPlyIoClasses(py::module &);
void defineIoPointCloudImageExtractorsClasses(py::module &);
void defineIoPngIoClasses(py::module &);
void defineIoRobotEyeGrabberClasses(py::module &);
void defineIoTarClasses(py::module &);
void defineIoVlpGrabberClasses(py::module &);
void defineIoVtkIoClasses(py::module &);
void defineIoVtkLibIoClasses(py::module &);


void defineIoClasses(py::module &m) {
    py::module m_io = m.def_submodule("io", "Submodule io");
    defineIoBoostClasses(m_io);
    defineIoAutoIoClasses(m_io);
    defineIoByteOrderClasses(m_io);
    defineIoDebayerClasses(m_io);
    defineIoEigenClasses(m_io);
    defineIoFileGrabberClasses(m_io);
    defineIoFileIoClasses(m_io);
    defineIoAsciiIoClasses(m_io);
    defineIoGrabberClasses(m_io);
    defineIoHdlGrabberClasses(m_io);
    defineIoIfsIoClasses(m_io);
    defineIoImageDepthClasses(m_io);
    defineIoImageGrabberClasses(m_io);
    defineIoImageMetadataWrapperClasses(m_io);
    defineIoImageClasses(m_io);
    defineIoImageIrClasses(m_io);
    defineIoImageRgb24Classes(m_io);
    defineIoImageYuv422Classes(m_io);
    defineIoIoClasses(m_io);
    defineIoIoExceptionClasses(m_io);
    defineIoIoOperatorsClasses(m_io);
    defineIoLzfClasses(m_io);
    defineIoLzfImageIoClasses(m_io);
    defineIoObjIoClasses(m_io);
    defineIoPcdGrabberClasses(m_io);
    defineIoPcdIoClasses(m_io);
    defineIoPlyClasses(m_io);
    defineIoPlyParserClasses(m_io);
    defineIoPlyIoClasses(m_io);
    defineIoPointCloudImageExtractorsClasses(m_io);
    defineIoPngIoClasses(m_io);
    defineIoRobotEyeGrabberClasses(m_io);
    defineIoTarClasses(m_io);
    defineIoVlpGrabberClasses(m_io);
    defineIoVtkIoClasses(m_io);
    defineIoVtkLibIoClasses(m_io);
}