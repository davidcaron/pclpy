
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "PCLPointCloud2.hpp"
#include "PointIndices.hpp"
#include "Vertices.hpp"
#include "PolygonMesh.hpp"
#include "TextureMesh.hpp"


void defineBaseClasses1(py::module &m) {
    definePCLPointCloud2Classes(m);
    definePointIndicesClasses(m);
    defineVerticesClasses(m);
    definePolygonMeshClasses(m);
    defineTextureMeshClasses(m);
}