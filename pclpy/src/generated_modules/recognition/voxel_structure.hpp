
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/voxel_structure.h>

using namespace pcl::recognition;


template<class T, typename REAL = float>
void defineRecognitionVoxelStructure(py::module &m, std::string const & suffix) {
    using Class = recognition::VoxelStructure<T, REAL>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("build", py::overload_cast<const REAL[6], int[3]> (&Class::build));
    cls.def("clear", &Class::clear);
    cls.def("compute3d_id", &Class::compute3dId);
    cls.def("compute_voxel_center", &Class::computeVoxelCenter);
    cls.def("get_voxel", py::overload_cast<const REAL[3]> (&Class::getVoxel));
    cls.def("get_voxel", py::overload_cast<int, int, int> (&Class::getVoxel, py::const_));
    cls.def("get_voxels", py::overload_cast<> (&Class::getVoxels, py::const_));
    cls.def("get_voxels", py::overload_cast<> (&Class::getVoxels));
    cls.def("get_bounds", py::overload_cast<> (&Class::getBounds, py::const_));
    cls.def("get_bounds", py::overload_cast<REAL[6]> (&Class::getBounds, py::const_));
        
}

void defineRecognitionVoxelStructureClasses(py::module &sub_module) {
}