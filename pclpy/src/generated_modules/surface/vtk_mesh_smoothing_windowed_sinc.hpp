
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>



void defineSurfaceMeshSmoothingWindowedSincVTK(py::module &m) {
    using Class = pcl::MeshSmoothingWindowedSincVTK;
    py::class_<Class, pcl::MeshProcessing, boost::shared_ptr<Class>> cls(m, "MeshSmoothingWindowedSincVTK");
    cls.def(py::init<>());
    cls.def("setNumIter", &Class::setNumIter, "num_iter"_a);
    cls.def("setPassBand", &Class::setPassBand, "pass_band"_a);
    cls.def("setNormalizeCoordinates", &Class::setNormalizeCoordinates, "normalize_coordinates"_a);
    cls.def("setFeatureEdgeSmoothing", &Class::setFeatureEdgeSmoothing, "feature_edge_smoothing"_a);
    cls.def("setFeatureAngle", &Class::setFeatureAngle, "feature_angle"_a);
    cls.def("setEdgeAngle", &Class::setEdgeAngle, "edge_angle"_a);
    cls.def("setBoundarySmoothing", &Class::setBoundarySmoothing, "boundary_smoothing"_a);
    cls.def("getNumIter", &Class::getNumIter);
    cls.def("getPassBand", &Class::getPassBand);
    cls.def("getNormalizeCoordinates", &Class::getNormalizeCoordinates);
    cls.def("getFeatureEdgeSmoothing", &Class::getFeatureEdgeSmoothing);
    cls.def("getFeatureAngle", &Class::getFeatureAngle);
    cls.def("getEdgeAngle", &Class::getEdgeAngle);
    cls.def("getBoundarySmoothing", &Class::getBoundarySmoothing);
}

void defineSurfaceVtkMeshSmoothingWindowedSincFunctions(py::module &m) {
}

void defineSurfaceVtkMeshSmoothingWindowedSincClasses(py::module &sub_module) {
    defineSurfaceMeshSmoothingWindowedSincVTK(sub_module);
}