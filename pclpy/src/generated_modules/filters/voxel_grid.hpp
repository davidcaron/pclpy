
#include <pcl/filters/voxel_grid.h>



template <typename PointT>
void defineFiltersVoxelGrid(py::module &m, std::string const & suffix) {
    using Class = pcl::VoxelGrid<PointT>;
    py::class_<Class, pcl::Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setLeafSize", py::overload_cast<const Eigen::Vector4f &> (&Class::setLeafSize), "leaf_size"_a);
    cls.def("setLeafSize", py::overload_cast<float, float, float> (&Class::setLeafSize), "lx"_a, "ly"_a, "lz"_a);
    cls.def("setDownsampleAllData", &Class::setDownsampleAllData, "downsample"_a);
    cls.def("setMinimumPointsNumberPerVoxel", &Class::setMinimumPointsNumberPerVoxel, "min_points_per_voxel"_a);
    cls.def("setSaveLeafLayout", &Class::setSaveLeafLayout, "save_leaf_layout"_a);
    cls.def("setFilterFieldName", &Class::setFilterFieldName, "field_name"_a);
    cls.def("setFilterLimits", &Class::setFilterLimits, "limit_min"_a, "limit_max"_a);
    cls.def("setFilterLimitsNegative", &Class::setFilterLimitsNegative, "limit_negative"_a);
    cls.def("getLeafSize", &Class::getLeafSize);
    cls.def("getDownsampleAllData", &Class::getDownsampleAllData);
    cls.def("getMinimumPointsNumberPerVoxel", &Class::getMinimumPointsNumberPerVoxel);
    cls.def("getSaveLeafLayout", &Class::getSaveLeafLayout);
    cls.def("getMinBoxCoordinates", &Class::getMinBoxCoordinates);
    cls.def("getMaxBoxCoordinates", &Class::getMaxBoxCoordinates);
    cls.def("getNrDivisions", &Class::getNrDivisions);
    cls.def("getDivisionMultiplier", &Class::getDivisionMultiplier);
    cls.def("getCentroidIndex", &Class::getCentroidIndex, "p"_a);
    cls.def("getNeighborCentroidIndices", &Class::getNeighborCentroidIndices, "reference_point"_a, "relative_coordinates"_a);
    cls.def("getLeafLayout", &Class::getLeafLayout);
    cls.def("getGridCoordinates", &Class::getGridCoordinates, "x"_a, "y"_a, "z"_a);
    cls.def("getCentroidIndexAt", &Class::getCentroidIndexAt, "ijk"_a);
    cls.def("getFilterFieldName", &Class::getFilterFieldName);
    cls.def("getFilterLimits", &Class::getFilterLimits, "limit_min"_a, "limit_max"_a);
    cls.def("getFilterLimitsNegative", py::overload_cast<bool &> (&Class::getFilterLimitsNegative), "limit_negative"_a);
    cls.def("getFilterLimitsNegative", py::overload_cast<> (&Class::getFilterLimitsNegative));
        
}

void defineFiltersVoxelGridFunctions(py::module &m) {
}

void defineFiltersVoxelGridClasses(py::module &sub_module) {
    py::module sub_module_VoxelGrid = sub_module.def_submodule("VoxelGrid", "Submodule VoxelGrid");
    defineFiltersVoxelGrid<pcl::InterestPoint>(sub_module_VoxelGrid, "InterestPoint");
    defineFiltersVoxelGrid<pcl::PointDEM>(sub_module_VoxelGrid, "PointDEM");
    defineFiltersVoxelGrid<pcl::PointNormal>(sub_module_VoxelGrid, "PointNormal");
    defineFiltersVoxelGrid<pcl::PointSurfel>(sub_module_VoxelGrid, "PointSurfel");
    defineFiltersVoxelGrid<pcl::PointWithRange>(sub_module_VoxelGrid, "PointWithRange");
    defineFiltersVoxelGrid<pcl::PointWithScale>(sub_module_VoxelGrid, "PointWithScale");
    defineFiltersVoxelGrid<pcl::PointWithViewpoint>(sub_module_VoxelGrid, "PointWithViewpoint");
    defineFiltersVoxelGrid<pcl::PointXYZ>(sub_module_VoxelGrid, "PointXYZ");
    defineFiltersVoxelGrid<pcl::PointXYZHSV>(sub_module_VoxelGrid, "PointXYZHSV");
    defineFiltersVoxelGrid<pcl::PointXYZI>(sub_module_VoxelGrid, "PointXYZI");
    defineFiltersVoxelGrid<pcl::PointXYZINormal>(sub_module_VoxelGrid, "PointXYZINormal");
    defineFiltersVoxelGrid<pcl::PointXYZL>(sub_module_VoxelGrid, "PointXYZL");
    defineFiltersVoxelGrid<pcl::PointXYZLNormal>(sub_module_VoxelGrid, "PointXYZLNormal");
    defineFiltersVoxelGrid<pcl::PointXYZRGB>(sub_module_VoxelGrid, "PointXYZRGB");
    defineFiltersVoxelGrid<pcl::PointXYZRGBA>(sub_module_VoxelGrid, "PointXYZRGBA");
    defineFiltersVoxelGrid<pcl::PointXYZRGBL>(sub_module_VoxelGrid, "PointXYZRGBL");
    defineFiltersVoxelGrid<pcl::PointXYZRGBNormal>(sub_module_VoxelGrid, "PointXYZRGBNormal");
    defineFiltersVoxelGridFunctions(sub_module);
}