
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/filters/voxel_grid_occlusion_estimation.h>



template <typename PointT>
void defineFiltersVoxelGridOcclusionEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::VoxelGridOcclusionEstimation<PointT>;
    py::class_<Class, pcl::VoxelGrid<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("initializeVoxelGrid", &Class::initializeVoxelGrid);
    cls.def("occlusionEstimation", py::overload_cast<int &, const Eigen::Vector3i &> (&Class::occlusionEstimation), "out_state"_a, "in_target_voxel"_a);
    cls.def("occlusionEstimation", py::overload_cast<int &, std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > &, const Eigen::Vector3i &> (&Class::occlusionEstimation), "out_state"_a, "out_ray"_a, "in_target_voxel"_a);
    cls.def("occlusionEstimationAll", &Class::occlusionEstimationAll, "occluded_voxels"_a);
    cls.def("getFilteredPointCloud", &Class::getFilteredPointCloud);
    cls.def("getMinBoundCoordinates", &Class::getMinBoundCoordinates);
    cls.def("getMaxBoundCoordinates", &Class::getMaxBoundCoordinates);
    cls.def("getCentroidCoordinate", &Class::getCentroidCoordinate, "ijk"_a);
        
}

void defineFiltersVoxelGridOcclusionEstimationFunctions(py::module &m) {
}

void defineFiltersVoxelGridOcclusionEstimationClasses(py::module &sub_module) {
    py::module sub_module_VoxelGridOcclusionEstimation = sub_module.def_submodule("VoxelGridOcclusionEstimation", "Submodule VoxelGridOcclusionEstimation");
    defineFiltersVoxelGridOcclusionEstimation<pcl::InterestPoint>(sub_module_VoxelGridOcclusionEstimation, "InterestPoint");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointDEM>(sub_module_VoxelGridOcclusionEstimation, "PointDEM");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointNormal>(sub_module_VoxelGridOcclusionEstimation, "PointNormal");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointSurfel>(sub_module_VoxelGridOcclusionEstimation, "PointSurfel");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointWithRange>(sub_module_VoxelGridOcclusionEstimation, "PointWithRange");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointWithScale>(sub_module_VoxelGridOcclusionEstimation, "PointWithScale");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointWithViewpoint>(sub_module_VoxelGridOcclusionEstimation, "PointWithViewpoint");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointXYZ>(sub_module_VoxelGridOcclusionEstimation, "PointXYZ");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointXYZHSV>(sub_module_VoxelGridOcclusionEstimation, "PointXYZHSV");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointXYZI>(sub_module_VoxelGridOcclusionEstimation, "PointXYZI");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointXYZINormal>(sub_module_VoxelGridOcclusionEstimation, "PointXYZINormal");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointXYZL>(sub_module_VoxelGridOcclusionEstimation, "PointXYZL");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointXYZLNormal>(sub_module_VoxelGridOcclusionEstimation, "PointXYZLNormal");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointXYZRGB>(sub_module_VoxelGridOcclusionEstimation, "PointXYZRGB");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointXYZRGBA>(sub_module_VoxelGridOcclusionEstimation, "PointXYZRGBA");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointXYZRGBL>(sub_module_VoxelGridOcclusionEstimation, "PointXYZRGBL");
    defineFiltersVoxelGridOcclusionEstimation<pcl::PointXYZRGBNormal>(sub_module_VoxelGridOcclusionEstimation, "PointXYZRGBNormal");
}