
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/voxel_grid_covariance.h>



template<typename PointT>
void defineFiltersVoxelGridCovariance(py::module &m, std::string const & suffix) {
    using Class = VoxelGridCovariance<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using LeafPtr = Class::LeafPtr;
    using LeafConstPtr = Class::LeafConstPtr;
    py::class_<Class, VoxelGrid<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("min_point_per_voxel", &Class::getMinPointPerVoxel, &Class::setMinPointPerVoxel);
    cls.def_property("cov_eig_value_inflation_ratio", &Class::getCovEigValueInflationRatio, &Class::setCovEigValueInflationRatio);
    cls.def("filter", py::overload_cast<Filter<PointT>::PointCloud &, bool> (&Class::filter));
    cls.def("filter", py::overload_cast<bool> (&Class::filter));
    cls.def("nearest_k_search", py::overload_cast<const PointT &, int, std::vector<LeafConstPtr> &, std::vector<float> &> (&Class::nearestKSearch));
    cls.def("nearest_k_search", py::overload_cast<const Filter<PointT>::PointCloud &, int, int, std::vector<LeafConstPtr> &, std::vector<float> &> (&Class::nearestKSearch));
    cls.def("radius_search", py::overload_cast<const PointT &, double, std::vector<LeafConstPtr> &, std::vector<float> &, unsigned int> (&Class::radiusSearch));
    cls.def("radius_search", py::overload_cast<const Filter<PointT>::PointCloud &, int, double, std::vector<LeafConstPtr> &, std::vector<float> &, unsigned int> (&Class::radiusSearch));
    cls.def("get_leaf", py::overload_cast<int> (&Class::getLeaf));
    cls.def("get_leaf", py::overload_cast<PointT &> (&Class::getLeaf));
    cls.def("get_leaf", py::overload_cast<Eigen::Vector3f &> (&Class::getLeaf));
        
}

void defineFiltersVoxelGridCovarianceClasses(py::module &sub_module) {
    py::module sub_module_VoxelGridCovariance = sub_module.def_submodule("VoxelGridCovariance", "Submodule VoxelGridCovariance");
    defineFiltersVoxelGridCovariance<InterestPoint>(sub_module_VoxelGridCovariance, "InterestPoint");
    defineFiltersVoxelGridCovariance<PointDEM>(sub_module_VoxelGridCovariance, "PointDEM");
    defineFiltersVoxelGridCovariance<PointNormal>(sub_module_VoxelGridCovariance, "PointNormal");
    defineFiltersVoxelGridCovariance<PointSurfel>(sub_module_VoxelGridCovariance, "PointSurfel");
    defineFiltersVoxelGridCovariance<PointWithRange>(sub_module_VoxelGridCovariance, "PointWithRange");
    defineFiltersVoxelGridCovariance<PointWithScale>(sub_module_VoxelGridCovariance, "PointWithScale");
    defineFiltersVoxelGridCovariance<PointWithViewpoint>(sub_module_VoxelGridCovariance, "PointWithViewpoint");
    defineFiltersVoxelGridCovariance<PointXYZ>(sub_module_VoxelGridCovariance, "PointXYZ");
    defineFiltersVoxelGridCovariance<PointXYZHSV>(sub_module_VoxelGridCovariance, "PointXYZHSV");
    defineFiltersVoxelGridCovariance<PointXYZI>(sub_module_VoxelGridCovariance, "PointXYZI");
    defineFiltersVoxelGridCovariance<PointXYZINormal>(sub_module_VoxelGridCovariance, "PointXYZINormal");
    defineFiltersVoxelGridCovariance<PointXYZL>(sub_module_VoxelGridCovariance, "PointXYZL");
    defineFiltersVoxelGridCovariance<PointXYZLNormal>(sub_module_VoxelGridCovariance, "PointXYZLNormal");
    defineFiltersVoxelGridCovariance<PointXYZRGB>(sub_module_VoxelGridCovariance, "PointXYZRGB");
    defineFiltersVoxelGridCovariance<PointXYZRGBA>(sub_module_VoxelGridCovariance, "PointXYZRGBA");
    defineFiltersVoxelGridCovariance<PointXYZRGBL>(sub_module_VoxelGridCovariance, "PointXYZRGBL");
    defineFiltersVoxelGridCovariance<PointXYZRGBNormal>(sub_module_VoxelGridCovariance, "PointXYZRGBNormal");
}