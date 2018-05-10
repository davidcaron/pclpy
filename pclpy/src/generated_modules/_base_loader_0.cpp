
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "correspondence.hpp"
#include "PCLHeader.hpp"


void defineBaseClasses(py::module &m) {
    defineCorrespondenceClasses(m);
    definePCLHeaderClasses(m);
}