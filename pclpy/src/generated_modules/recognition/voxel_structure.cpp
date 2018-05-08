
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

#include <pcl/recognition/voxel_structure.h>

using namespace pcl::recognition;


template<class T, typename REAL = float>
void defineRecognitionVoxelStructure(py::module &m, std::string const & suffix) {
    using Class = pcl::recognition::VoxelStructure<T, REAL>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("build", py::overload_cast<const REAL[6], int[3]> (&Class::build), "bounds"_a, "num_of_voxels"_a);
    cls.def("clear", &Class::clear);
    cls.def("compute3dId", &Class::compute3dId, "linear_id"_a, "id3d"_a);
    cls.def("computeVoxelCenter", &Class::computeVoxelCenter, "id3"_a, "center"_a);
    cls.def("getVoxel", py::overload_cast<const REAL[3]> (&Class::getVoxel), "p"_a);
    cls.def("getVoxel", py::overload_cast<int, int, int> (&Class::getVoxel, py::const_), "x"_a, "y"_a, "z"_a);
    cls.def("getVoxels", py::overload_cast<> (&Class::getVoxels, py::const_));
    cls.def("getVoxels", py::overload_cast<> (&Class::getVoxels));
    cls.def("getNumberOfVoxelsXYZ", &Class::getNumberOfVoxelsXYZ);
    cls.def("getNumberOfVoxels", &Class::getNumberOfVoxels);
    cls.def("getBounds", py::overload_cast<> (&Class::getBounds, py::const_));
    cls.def("getBounds", py::overload_cast<REAL[6]> (&Class::getBounds, py::const_), "b"_a);
    cls.def("getVoxelSpacing", &Class::getVoxelSpacing);
        
}

void defineRecognitionVoxelStructureFunctions(py::module &m) {
}

void defineRecognitionVoxelStructureClasses(py::module &sub_module) {
}