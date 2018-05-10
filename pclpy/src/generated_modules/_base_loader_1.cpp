
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "PCLImage.hpp"
#include "PCLPointField.hpp"
#include "PCLPointCloud2.hpp"


void defineBaseClasses1(py::module &m) {
    definePCLImageClasses(m);
    definePCLPointFieldClasses(m);
    definePCLPointCloud2Classes(m);
}