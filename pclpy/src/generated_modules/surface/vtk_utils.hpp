
#include <pcl/surface/vtk_smoothing/vtk_utils.h>



void defineSurfaceVTKUtils(py::module &m) {
    using Class = pcl::VTKUtils;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "VTKUtils");
    cls.def_static("convertToVTK", &Class::convertToVTK, "triangles"_a, "triangles_out_vtk"_a);
    cls.def_static("convertToPCL", &Class::convertToPCL, "vtk_polygons"_a, "triangles"_a);
    cls.def_static("vtk2mesh", &Class::vtk2mesh, "poly_data"_a, "mesh"_a);
    cls.def_static("mesh2vtk", &Class::mesh2vtk, "mesh"_a, "poly_data"_a);
}

void defineSurfaceVtkUtilsFunctions(py::module &m) {
}

void defineSurfaceVtkUtilsClasses(py::module &sub_module) {
    defineSurfaceVTKUtils(sub_module);
}