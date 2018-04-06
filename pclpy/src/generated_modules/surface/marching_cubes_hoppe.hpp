
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/marching_cubes_hoppe.h>



template <typename PointNT>
void defineSurfaceMarchingCubesHoppe(py::module &m, std::string const & suffix) {
    using Class = MarchingCubesHoppe<PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    py::class_<Class, MarchingCubes<PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("voxelize_data", &Class::voxelizeData);
        
}

void defineSurfaceMarchingCubesHoppeClasses(py::module &sub_module) {
    py::module sub_module_MarchingCubesHoppe = sub_module.def_submodule("MarchingCubesHoppe", "Submodule MarchingCubesHoppe");
    defineSurfaceMarchingCubesHoppe<PointNormal>(sub_module_MarchingCubesHoppe, "PointNormal");
    defineSurfaceMarchingCubesHoppe<PointXYZINormal>(sub_module_MarchingCubesHoppe, "PointXYZINormal");
    defineSurfaceMarchingCubesHoppe<PointXYZRGBNormal>(sub_module_MarchingCubesHoppe, "PointXYZRGBNormal");
}