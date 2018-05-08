
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>



void defineSurfaceMeshSmoothingLaplacianVTK(py::module &m) {
    using Class = pcl::MeshSmoothingLaplacianVTK;
    py::class_<Class, pcl::MeshProcessing, boost::shared_ptr<Class>> cls(m, "MeshSmoothingLaplacianVTK");
    cls.def(py::init<>());
    cls.def("setNumIter", &Class::setNumIter, "num_iter"_a);
    cls.def("setConvergence", &Class::setConvergence, "convergence"_a);
    cls.def("setRelaxationFactor", &Class::setRelaxationFactor, "relaxation_factor"_a);
    cls.def("setFeatureEdgeSmoothing", &Class::setFeatureEdgeSmoothing, "feature_edge_smoothing"_a);
    cls.def("setFeatureAngle", &Class::setFeatureAngle, "feature_angle"_a);
    cls.def("setEdgeAngle", &Class::setEdgeAngle, "edge_angle"_a);
    cls.def("setBoundarySmoothing", &Class::setBoundarySmoothing, "boundary_smoothing"_a);
    cls.def("getNumIter", &Class::getNumIter);
    cls.def("getConvergence", &Class::getConvergence);
    cls.def("getRelaxationFactor", &Class::getRelaxationFactor);
    cls.def("getFeatureEdgeSmoothing", &Class::getFeatureEdgeSmoothing);
    cls.def("getFeatureAngle", &Class::getFeatureAngle);
    cls.def("getEdgeAngle", &Class::getEdgeAngle);
    cls.def("getBoundarySmoothing", &Class::getBoundarySmoothing);
}

void defineSurfaceVtkMeshSmoothingLaplacianFunctions(py::module &m) {
}

void defineSurfaceVtkMeshSmoothingLaplacianClasses(py::module &sub_module) {
    defineSurfaceMeshSmoothingLaplacianVTK(sub_module);
}