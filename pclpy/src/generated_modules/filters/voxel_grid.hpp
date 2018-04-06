
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/voxel_grid.h>



template <typename PointT>
void defineFiltersVoxelGrid(py::module &m, std::string const & suffix) {
    using Class = VoxelGrid<PointT>;
    py::class_<Class, Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("downsample_all_data", &Class::getDownsampleAllData, &Class::setDownsampleAllData);
    cls.def_property("minimum_points_number_per_voxel", &Class::getMinimumPointsNumberPerVoxel, &Class::setMinimumPointsNumberPerVoxel);
    cls.def_property("save_leaf_layout", &Class::getSaveLeafLayout, &Class::setSaveLeafLayout);
    cls.def_property("filter_field_name", &Class::getFilterFieldName, &Class::setFilterFieldName);
    cls.def_property("filter_limits", &Class::getFilterLimits, &Class::setFilterLimits);
    cls.def("set_filter_limits_negative", &Class::setFilterLimitsNegative);
    cls.def("set_leaf_size", py::overload_cast<const Eigen::Vector4f &> (&Class::setLeafSize));
    cls.def("set_leaf_size", py::overload_cast<float, float, float> (&Class::setLeafSize));
    cls.def("get_filter_limits_negative", py::overload_cast<bool &> (&Class::getFilterLimitsNegative));
    cls.def("get_filter_limits_negative", py::overload_cast<> (&Class::getFilterLimitsNegative));
        
}

void defineFiltersVoxelGridClasses(py::module &sub_module) {
    py::module sub_module_VoxelGrid = sub_module.def_submodule("VoxelGrid", "Submodule VoxelGrid");
    defineFiltersVoxelGrid<InterestPoint>(sub_module_VoxelGrid, "InterestPoint");
    defineFiltersVoxelGrid<PointDEM>(sub_module_VoxelGrid, "PointDEM");
    defineFiltersVoxelGrid<PointNormal>(sub_module_VoxelGrid, "PointNormal");
    defineFiltersVoxelGrid<PointSurfel>(sub_module_VoxelGrid, "PointSurfel");
    defineFiltersVoxelGrid<PointWithRange>(sub_module_VoxelGrid, "PointWithRange");
    defineFiltersVoxelGrid<PointWithScale>(sub_module_VoxelGrid, "PointWithScale");
    defineFiltersVoxelGrid<PointWithViewpoint>(sub_module_VoxelGrid, "PointWithViewpoint");
    defineFiltersVoxelGrid<PointXYZ>(sub_module_VoxelGrid, "PointXYZ");
    defineFiltersVoxelGrid<PointXYZHSV>(sub_module_VoxelGrid, "PointXYZHSV");
    defineFiltersVoxelGrid<PointXYZI>(sub_module_VoxelGrid, "PointXYZI");
    defineFiltersVoxelGrid<PointXYZINormal>(sub_module_VoxelGrid, "PointXYZINormal");
    defineFiltersVoxelGrid<PointXYZL>(sub_module_VoxelGrid, "PointXYZL");
    defineFiltersVoxelGrid<PointXYZLNormal>(sub_module_VoxelGrid, "PointXYZLNormal");
    defineFiltersVoxelGrid<PointXYZRGB>(sub_module_VoxelGrid, "PointXYZRGB");
    defineFiltersVoxelGrid<PointXYZRGBA>(sub_module_VoxelGrid, "PointXYZRGBA");
    defineFiltersVoxelGrid<PointXYZRGBL>(sub_module_VoxelGrid, "PointXYZRGBL");
    defineFiltersVoxelGrid<PointXYZRGBNormal>(sub_module_VoxelGrid, "PointXYZRGBNormal");
}