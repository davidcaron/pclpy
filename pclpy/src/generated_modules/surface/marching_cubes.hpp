
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/marching_cubes.h>



template <typename PointNT>
void defineSurfaceMarchingCubes(py::module &m, std::string const & suffix) {
    using Class = MarchingCubes<PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    py::class_<Class, SurfaceReconstruction<PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("iso_level", &Class::getIsoLevel, &Class::setIsoLevel);
    cls.def_property("grid_resolution", &Class::getGridResolution, &Class::setGridResolution);
    cls.def_property("percentage_extend_grid", &Class::getPercentageExtendGrid, &Class::setPercentageExtendGrid);
        
}

void defineSurfaceMarchingCubesClasses(py::module &sub_module) {
    py::module sub_module_MarchingCubes = sub_module.def_submodule("MarchingCubes", "Submodule MarchingCubes");
    defineSurfaceMarchingCubes<PointNormal>(sub_module_MarchingCubes, "PointNormal");
    defineSurfaceMarchingCubes<PointXYZINormal>(sub_module_MarchingCubes, "PointXYZINormal");
    defineSurfaceMarchingCubes<PointXYZRGBNormal>(sub_module_MarchingCubes, "PointXYZRGBNormal");
}