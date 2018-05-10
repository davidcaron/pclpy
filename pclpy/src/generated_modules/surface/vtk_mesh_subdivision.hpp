
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>



void defineSurfaceMeshSubdivisionVTK(py::module &m) {
    using Class = pcl::MeshSubdivisionVTK;
    py::class_<Class, pcl::MeshProcessing, boost::shared_ptr<Class>> cls(m, "MeshSubdivisionVTK");
    py::enum_<Class::MeshSubdivisionVTKFilterType>(cls, "MeshSubdivisionVTKFilterType")
        .value("LINEAR", Class::MeshSubdivisionVTKFilterType::LINEAR)
        .value("LOOP", Class::MeshSubdivisionVTKFilterType::LOOP)
        .value("BUTTERFLY", Class::MeshSubdivisionVTKFilterType::BUTTERFLY)
        .export_values();
    cls.def(py::init<>());
    cls.def("setFilterType", &Class::setFilterType, "type"_a);
    cls.def("getFilterType", &Class::getFilterType);
}

void defineSurfaceVtkMeshSubdivisionFunctions(py::module &m) {
}

void defineSurfaceVtkMeshSubdivisionClasses(py::module &sub_module) {
    defineSurfaceMeshSubdivisionVTK(sub_module);
}