
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "2d/convolution.hpp"


void define2dClasses(py::module &m) {
    py::module m_2d = m.def_submodule("module_2d", "Submodule 2d");
    define2dConvolutionClasses(m_2d);
}