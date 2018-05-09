
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/geometry/polygon_operations.h>



void defineGeometryPolygonOperationsFunctions(py::module &m) {
}

void defineGeometryPolygonOperationsClasses(py::module &sub_module) {
    defineGeometryPolygonOperationsFunctions(sub_module);
}