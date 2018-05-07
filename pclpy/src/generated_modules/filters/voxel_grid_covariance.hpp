
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/filters/voxel_grid_covariance.h>



template<typename PointT>
void defineFiltersVoxelGridCovariance(py::module &m, std::string const & suffix) {
    using Class = pcl::VoxelGridCovariance<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using LeafPtr = Class::LeafPtr;
    using LeafConstPtr = Class::LeafConstPtr;
    py::class_<Class, pcl::VoxelGrid<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("filter", py::overload_cast<pcl::Filter<PointT>::PointCloud &, bool> (&Class::filter), "output"_a, "searchable"_a=false);
    cls.def("filter", py::overload_cast<bool> (&Class::filter), "searchable"_a=false);
    cls.def("nearestKSearch", py::overload_cast<const PointT &, int, std::vector<LeafConstPtr> &, std::vector<float> &> (&Class::nearestKSearch), "point"_a, "k"_a, "k_leaves"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<const pcl::Filter<PointT>::PointCloud &, int, int, std::vector<LeafConstPtr> &, std::vector<float> &> (&Class::nearestKSearch), "cloud"_a, "index"_a, "k"_a, "k_leaves"_a, "k_sqr_distances"_a);
    cls.def("radiusSearch", py::overload_cast<const PointT &, double, std::vector<LeafConstPtr> &, std::vector<float> &, unsigned int> (&Class::radiusSearch), "point"_a, "radius"_a, "k_leaves"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<const pcl::Filter<PointT>::PointCloud &, int, double, std::vector<LeafConstPtr> &, std::vector<float> &, unsigned int> (&Class::radiusSearch), "cloud"_a, "index"_a, "radius"_a, "k_leaves"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("setMinPointPerVoxel", &Class::setMinPointPerVoxel, "min_points_per_voxel"_a);
    cls.def("setCovEigValueInflationRatio", &Class::setCovEigValueInflationRatio, "min_covar_eigvalue_mult"_a);
    cls.def("getMinPointPerVoxel", &Class::getMinPointPerVoxel);
    cls.def("getCovEigValueInflationRatio", &Class::getCovEigValueInflationRatio);
    cls.def("getLeaf", py::overload_cast<int> (&Class::getLeaf), "index"_a);
    cls.def("getLeaf", py::overload_cast<PointT &> (&Class::getLeaf), "p"_a);
    cls.def("getLeaf", py::overload_cast<Eigen::Vector3f &> (&Class::getLeaf), "p"_a);
    cls.def("getNeighborhoodAtPoint", &Class::getNeighborhoodAtPoint, "reference_point"_a, "neighbors"_a);
    cls.def("getLeaves", &Class::getLeaves);
    cls.def("getCentroids", &Class::getCentroids);
    cls.def("getDisplayCloud", &Class::getDisplayCloud, "cell_cloud"_a);
        
}

void defineFiltersVoxelGridCovarianceFunctions(py::module &m) {
}

void defineFiltersVoxelGridCovarianceClasses(py::module &sub_module) {
    py::module sub_module_VoxelGridCovariance = sub_module.def_submodule("VoxelGridCovariance", "Submodule VoxelGridCovariance");
    defineFiltersVoxelGridCovariance<pcl::InterestPoint>(sub_module_VoxelGridCovariance, "InterestPoint");
    defineFiltersVoxelGridCovariance<pcl::PointDEM>(sub_module_VoxelGridCovariance, "PointDEM");
    defineFiltersVoxelGridCovariance<pcl::PointNormal>(sub_module_VoxelGridCovariance, "PointNormal");
    defineFiltersVoxelGridCovariance<pcl::PointSurfel>(sub_module_VoxelGridCovariance, "PointSurfel");
    defineFiltersVoxelGridCovariance<pcl::PointWithRange>(sub_module_VoxelGridCovariance, "PointWithRange");
    defineFiltersVoxelGridCovariance<pcl::PointWithScale>(sub_module_VoxelGridCovariance, "PointWithScale");
    defineFiltersVoxelGridCovariance<pcl::PointWithViewpoint>(sub_module_VoxelGridCovariance, "PointWithViewpoint");
    defineFiltersVoxelGridCovariance<pcl::PointXYZ>(sub_module_VoxelGridCovariance, "PointXYZ");
    defineFiltersVoxelGridCovariance<pcl::PointXYZHSV>(sub_module_VoxelGridCovariance, "PointXYZHSV");
    defineFiltersVoxelGridCovariance<pcl::PointXYZI>(sub_module_VoxelGridCovariance, "PointXYZI");
    defineFiltersVoxelGridCovariance<pcl::PointXYZINormal>(sub_module_VoxelGridCovariance, "PointXYZINormal");
    defineFiltersVoxelGridCovariance<pcl::PointXYZL>(sub_module_VoxelGridCovariance, "PointXYZL");
    defineFiltersVoxelGridCovariance<pcl::PointXYZLNormal>(sub_module_VoxelGridCovariance, "PointXYZLNormal");
    defineFiltersVoxelGridCovariance<pcl::PointXYZRGB>(sub_module_VoxelGridCovariance, "PointXYZRGB");
    defineFiltersVoxelGridCovariance<pcl::PointXYZRGBA>(sub_module_VoxelGridCovariance, "PointXYZRGBA");
    defineFiltersVoxelGridCovariance<pcl::PointXYZRGBL>(sub_module_VoxelGridCovariance, "PointXYZRGBL");
    defineFiltersVoxelGridCovariance<pcl::PointXYZRGBNormal>(sub_module_VoxelGridCovariance, "PointXYZRGBNormal");
}