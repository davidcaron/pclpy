
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "PolygonMesh.hpp"
#include "TextureMesh.hpp"
#include "point_traits.hpp"
#include "point_cloud.hpp"


void defineBaseClasses2(py::module &m) {
    definePolygonMeshClasses(m);
    defineTextureMeshClasses(m);
    definePointTraitsClasses(m);
    definePointCloudClasses(m);
}