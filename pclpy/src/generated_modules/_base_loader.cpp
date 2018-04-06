
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"
#include "pcl_config.hpp"
#include "pcl_exports.hpp"
#include "correspondence.hpp"
#include "pcl_macros.hpp"
#include "PCLHeader.hpp"
#include "ModelCoefficients.hpp"
#include "PCLImage.hpp"
#include "PCLPointField.hpp"
#include "PCLPointCloud2.hpp"
#include "PointIndices.hpp"
#include "Vertices.hpp"
#include "PolygonMesh.hpp"
#include "TextureMesh.hpp"
#include "point_traits.hpp"
#include "point_cloud.hpp"
#include "cloud_iterator.hpp"
#include "conversions.hpp"
#include "pcl_base.hpp"
#include "point_types_conversion.hpp"
#include "register_point_struct.hpp"
#include "point_types.hpp"
#include "point_representation.hpp"
#include "sse.hpp"


void defineBaseClasses(py::module &m) {
    definePclConfigClasses(m);
    definePclExportsClasses(m);
    defineCorrespondenceClasses(m);
    definePclMacrosClasses(m);
    definePCLHeaderClasses(m);
    defineModelCoefficientsClasses(m);
    definePCLImageClasses(m);
    definePCLPointFieldClasses(m);
    definePCLPointCloud2Classes(m);
    definePointIndicesClasses(m);
    defineVerticesClasses(m);
    definePolygonMeshClasses(m);
    defineTextureMeshClasses(m);
    definePointTraitsClasses(m);
    definePointCloudClasses(m);
    defineCloudIteratorClasses(m);
    defineConversionsClasses(m);
    definePclBaseClasses(m);
    definePointTypesConversionClasses(m);
    defineRegisterPointStructClasses(m);
    definePointTypesClasses(m);
    definePointRepresentationClasses(m);
    defineSseClasses(m);
}