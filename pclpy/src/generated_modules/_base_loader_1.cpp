
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
#include "pcl_base.hpp"
#include "point_types_conversion.hpp"
#include "point_types.hpp"


void defineBaseClasses1(py::module &m) {
    definePolygonMeshClasses(m);
    defineTextureMeshClasses(m);
    definePointTraitsClasses(m);
    definePointCloudClasses(m);
    definePclBaseClasses(m);
    definePointTypesConversionClasses(m);
    definePointTypesClasses(m);
}