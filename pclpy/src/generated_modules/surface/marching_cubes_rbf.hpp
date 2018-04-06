
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/marching_cubes_rbf.h>



template <typename PointNT>
void defineSurfaceMarchingCubesRBF(py::module &m, std::string const & suffix) {
    using Class = MarchingCubesRBF<PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    py::class_<Class, MarchingCubes<PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("off_surface_displacement", &Class::getOffSurfaceDisplacement, &Class::setOffSurfaceDisplacement);
    cls.def("voxelize_data", &Class::voxelizeData);
        
}

void defineSurfaceMarchingCubesRbfClasses(py::module &sub_module) {
    py::module sub_module_MarchingCubesRBF = sub_module.def_submodule("MarchingCubesRBF", "Submodule MarchingCubesRBF");
    defineSurfaceMarchingCubesRBF<PointNormal>(sub_module_MarchingCubesRBF, "PointNormal");
    defineSurfaceMarchingCubesRBF<PointXYZINormal>(sub_module_MarchingCubesRBF, "PointXYZINormal");
    defineSurfaceMarchingCubesRBF<PointXYZRGBNormal>(sub_module_MarchingCubesRBF, "PointXYZRGBNormal");
}