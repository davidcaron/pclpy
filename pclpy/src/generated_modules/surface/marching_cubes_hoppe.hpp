
#include <pcl/surface/marching_cubes_hoppe.h>



template <typename PointNT>
void defineSurfaceMarchingCubesHoppe(py::module &m, std::string const & suffix) {
    using Class = pcl::MarchingCubesHoppe<PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    py::class_<Class, pcl::MarchingCubes<PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("voxelizeData", &Class::voxelizeData);
        
}

void defineSurfaceMarchingCubesHoppeFunctions(py::module &m) {
}

void defineSurfaceMarchingCubesHoppeClasses(py::module &sub_module) {
    py::module sub_module_MarchingCubesHoppe = sub_module.def_submodule("MarchingCubesHoppe", "Submodule MarchingCubesHoppe");
    defineSurfaceMarchingCubesHoppe<pcl::PointNormal>(sub_module_MarchingCubesHoppe, "PointNormal");
    defineSurfaceMarchingCubesHoppe<pcl::PointXYZINormal>(sub_module_MarchingCubesHoppe, "PointXYZINormal");
    defineSurfaceMarchingCubesHoppe<pcl::PointXYZRGBNormal>(sub_module_MarchingCubesHoppe, "PointXYZRGBNormal");
}