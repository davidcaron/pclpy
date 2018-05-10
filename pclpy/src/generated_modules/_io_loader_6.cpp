
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "io/ply.hpp"
#include "io/ply_io.hpp"
#include "io/robot_eye_grabber.hpp"


void defineIoClasses6(py::module &m) {
    py::module m_io = m.def_submodule("io", "Submodule io");
    defineIoPlyClasses(m_io);
    defineIoPlyIoClasses(m_io);
    defineIoRobotEyeGrabberClasses(m_io);
}