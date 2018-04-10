
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/TextureMesh.h>



void defineTexMaterial(py::module &m) {
    using Class = pcl::TexMaterial;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "TexMaterial");
    cls.def(py::init<>());
    cls.def_readonly("tex_name", &Class::tex_name);
    cls.def_readonly("tex_file", &Class::tex_file);
    cls.def_readonly("tex_ka", &Class::tex_Ka);
    cls.def_readonly("tex_kd", &Class::tex_Kd);
    cls.def_readonly("tex_ks", &Class::tex_Ks);
    cls.def_readonly("tex_d", &Class::tex_d);
    cls.def_readonly("tex_ns", &Class::tex_Ns);
    cls.def_readonly("tex_illum", &Class::tex_illum);
}

void defineTextureMesh(py::module &m) {
    using Class = pcl::TextureMesh;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "TextureMesh");
    cls.def(py::init<>());
    cls.def_readonly("cloud", &Class::cloud);
    cls.def_readonly("header", &Class::header);
    cls.def_readonly("tex_polygons", &Class::tex_polygons);
    cls.def_readonly("tex_coordinates", &Class::tex_coordinates);
    cls.def_readonly("tex_materials", &Class::tex_materials);
}

void defineTextureMeshClasses(py::module &sub_module) {
    defineTexMaterial(sub_module);
    defineTextureMesh(sub_module);
}