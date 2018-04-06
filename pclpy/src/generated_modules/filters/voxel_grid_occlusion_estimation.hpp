
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/voxel_grid_occlusion_estimation.h>



template <typename PointT>
void defineFiltersVoxelGridOcclusionEstimation(py::module &m, std::string const & suffix) {
    using Class = VoxelGridOcclusionEstimation<PointT>;
    py::class_<Class, VoxelGrid<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("occlusion_estimation", py::overload_cast<int &, const Eigen::Vector3i &> (&Class::occlusionEstimation));
    cls.def("occlusion_estimation", py::overload_cast<int &, std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > &, const Eigen::Vector3i &> (&Class::occlusionEstimation));
    cls.def("initialize_voxel_grid", &Class::initializeVoxelGrid);
    cls.def("occlusion_estimation_all", &Class::occlusionEstimationAll);
        
}

void defineFiltersVoxelGridOcclusionEstimationClasses(py::module &sub_module) {
    py::module sub_module_VoxelGridOcclusionEstimation = sub_module.def_submodule("VoxelGridOcclusionEstimation", "Submodule VoxelGridOcclusionEstimation");
    defineFiltersVoxelGridOcclusionEstimation<InterestPoint>(sub_module_VoxelGridOcclusionEstimation, "InterestPoint");
    defineFiltersVoxelGridOcclusionEstimation<PointDEM>(sub_module_VoxelGridOcclusionEstimation, "PointDEM");
    defineFiltersVoxelGridOcclusionEstimation<PointNormal>(sub_module_VoxelGridOcclusionEstimation, "PointNormal");
    defineFiltersVoxelGridOcclusionEstimation<PointSurfel>(sub_module_VoxelGridOcclusionEstimation, "PointSurfel");
    defineFiltersVoxelGridOcclusionEstimation<PointWithRange>(sub_module_VoxelGridOcclusionEstimation, "PointWithRange");
    defineFiltersVoxelGridOcclusionEstimation<PointWithScale>(sub_module_VoxelGridOcclusionEstimation, "PointWithScale");
    defineFiltersVoxelGridOcclusionEstimation<PointWithViewpoint>(sub_module_VoxelGridOcclusionEstimation, "PointWithViewpoint");
    defineFiltersVoxelGridOcclusionEstimation<PointXYZ>(sub_module_VoxelGridOcclusionEstimation, "PointXYZ");
    defineFiltersVoxelGridOcclusionEstimation<PointXYZHSV>(sub_module_VoxelGridOcclusionEstimation, "PointXYZHSV");
    defineFiltersVoxelGridOcclusionEstimation<PointXYZI>(sub_module_VoxelGridOcclusionEstimation, "PointXYZI");
    defineFiltersVoxelGridOcclusionEstimation<PointXYZINormal>(sub_module_VoxelGridOcclusionEstimation, "PointXYZINormal");
    defineFiltersVoxelGridOcclusionEstimation<PointXYZL>(sub_module_VoxelGridOcclusionEstimation, "PointXYZL");
    defineFiltersVoxelGridOcclusionEstimation<PointXYZLNormal>(sub_module_VoxelGridOcclusionEstimation, "PointXYZLNormal");
    defineFiltersVoxelGridOcclusionEstimation<PointXYZRGB>(sub_module_VoxelGridOcclusionEstimation, "PointXYZRGB");
    defineFiltersVoxelGridOcclusionEstimation<PointXYZRGBA>(sub_module_VoxelGridOcclusionEstimation, "PointXYZRGBA");
    defineFiltersVoxelGridOcclusionEstimation<PointXYZRGBL>(sub_module_VoxelGridOcclusionEstimation, "PointXYZRGBL");
    defineFiltersVoxelGridOcclusionEstimation<PointXYZRGBNormal>(sub_module_VoxelGridOcclusionEstimation, "PointXYZRGBNormal");
}