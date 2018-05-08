
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

void define2dMorphologyClasses(py::module &);
void define2dConvolutionClasses(py::module &);


void define2dClasses(py::module &m) {
    py::module m_2d = m.def_submodule("module_2d", "Submodule 2d");
    define2dMorphologyClasses(m_2d);
    define2dConvolutionClasses(m_2d);
}