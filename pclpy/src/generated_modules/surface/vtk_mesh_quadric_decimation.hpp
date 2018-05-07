
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>



void defineSurfaceMeshQuadricDecimationVTK(py::module &m) {
    using Class = pcl::MeshQuadricDecimationVTK;
    py::class_<Class, pcl::MeshProcessing, boost::shared_ptr<Class>> cls(m, "MeshQuadricDecimationVTK");
    cls.def(py::init<>());
    cls.def("setTargetReductionFactor", &Class::setTargetReductionFactor, "factor"_a);
    cls.def("getTargetReductionFactor", &Class::getTargetReductionFactor);
}

void defineSurfaceVtkMeshQuadricDecimationFunctions(py::module &m) {
}

void defineSurfaceVtkMeshQuadricDecimationClasses(py::module &sub_module) {
    defineSurfaceMeshQuadricDecimationVTK(sub_module);
}