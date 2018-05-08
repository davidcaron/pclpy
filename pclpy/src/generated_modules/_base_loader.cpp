
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

void definePclConfigClasses(py::module &);
void definePclExportsClasses(py::module &);
void defineCorrespondenceClasses(py::module &);
void definePclMacrosClasses(py::module &);
void definePCLHeaderClasses(py::module &);
void defineModelCoefficientsClasses(py::module &);
void definePCLImageClasses(py::module &);
void definePCLPointFieldClasses(py::module &);
void definePCLPointCloud2Classes(py::module &);
void definePointIndicesClasses(py::module &);
void defineVerticesClasses(py::module &);
void definePolygonMeshClasses(py::module &);
void defineTextureMeshClasses(py::module &);
void definePointTraitsClasses(py::module &);
void definePointCloudClasses(py::module &);
void defineCloudIteratorClasses(py::module &);
void definePclBaseClasses(py::module &);
void definePointTypesConversionClasses(py::module &);
void defineRegisterPointStructClasses(py::module &);
void definePointTypesClasses(py::module &);


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
    definePclBaseClasses(m);
    definePointTypesConversionClasses(m);
    defineRegisterPointStructClasses(m);
    definePointTypesClasses(m);
}