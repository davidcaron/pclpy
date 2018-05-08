
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

#include <pcl/TextureMesh.h>



void defineTexMaterial(py::module &m) {
    using Class = pcl::TexMaterial;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "TexMaterial");
    cls.def(py::init<>());
    cls.def_readwrite("tex_name", &Class::tex_name);
    cls.def_readwrite("tex_file", &Class::tex_file);
    cls.def_readwrite("tex_Ka", &Class::tex_Ka);
    cls.def_readwrite("tex_Kd", &Class::tex_Kd);
    cls.def_readwrite("tex_Ks", &Class::tex_Ks);
    cls.def_readwrite("tex_d", &Class::tex_d);
    cls.def_readwrite("tex_Ns", &Class::tex_Ns);
    cls.def_readwrite("tex_illum", &Class::tex_illum);
}

void defineTextureMesh(py::module &m) {
    using Class = pcl::TextureMesh;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "TextureMesh");
    cls.def(py::init<>());
    cls.def_readwrite("cloud", &Class::cloud);
    cls.def_readwrite("header", &Class::header);
    cls.def_readwrite("tex_polygons", &Class::tex_polygons);
    cls.def_readwrite("tex_coordinates", &Class::tex_coordinates);
    cls.def_readwrite("tex_materials", &Class::tex_materials);
}

void defineTextureMeshFunctions(py::module &m) {
}

void defineTextureMeshClasses(py::module &sub_module) {
    defineTexMaterial(sub_module);
    defineTextureMesh(sub_module);
}