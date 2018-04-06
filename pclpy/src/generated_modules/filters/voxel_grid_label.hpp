
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/voxel_grid_label.h>



void defineFiltersVoxelGridLabel(py::module &m) {
    using Class = VoxelGridLabel;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, VoxelGrid<PointXYZRGBL>, boost::shared_ptr<Class>> cls(m, "VoxelGridLabel");
    cls.def(py::init<>());
}

void defineFiltersVoxelGridLabelClasses(py::module &sub_module) {
    defineFiltersVoxelGridLabel(sub_module);
}