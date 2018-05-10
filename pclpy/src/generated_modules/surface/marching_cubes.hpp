
#include <pcl/surface/marching_cubes.h>



template <typename PointNT>
void defineSurfaceMarchingCubes(py::module &m, std::string const & suffix) {
    using Class = pcl::MarchingCubes<PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    py::class_<Class, pcl::SurfaceReconstruction<PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("setIsoLevel", &Class::setIsoLevel, "iso_level"_a);
    cls.def("setGridResolution", &Class::setGridResolution, "res_x"_a, "res_y"_a, "res_z"_a);
    cls.def("setPercentageExtendGrid", &Class::setPercentageExtendGrid, "percentage"_a);
    cls.def("getIsoLevel", &Class::getIsoLevel);
    cls.def("getGridResolution", &Class::getGridResolution, "res_x"_a, "res_y"_a, "res_z"_a);
    cls.def("getPercentageExtendGrid", &Class::getPercentageExtendGrid);
        
}

void defineSurfaceMarchingCubesFunctions(py::module &m) {
}

void defineSurfaceMarchingCubesClasses(py::module &sub_module) {
    py::module sub_module_MarchingCubes = sub_module.def_submodule("MarchingCubes", "Submodule MarchingCubes");
    defineSurfaceMarchingCubes<pcl::PointNormal>(sub_module_MarchingCubes, "PointNormal");
    defineSurfaceMarchingCubes<pcl::PointXYZINormal>(sub_module_MarchingCubes, "PointXYZINormal");
    defineSurfaceMarchingCubes<pcl::PointXYZRGBNormal>(sub_module_MarchingCubes, "PointXYZRGBNormal");
}