
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/filters/approximate_voxel_grid.h>



template <typename PointT>
void defineFiltersApproximateVoxelGrid(py::module &m, std::string const & suffix) {
    using Class = pcl::ApproximateVoxelGrid<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    // Operators not implemented (operator=);
    cls.def("setLeafSize", py::overload_cast<const Eigen::Vector3f &> (&Class::setLeafSize), "leaf_size"_a);
    cls.def("setLeafSize", py::overload_cast<float, float, float> (&Class::setLeafSize), "lx"_a, "ly"_a, "lz"_a);
    cls.def("setDownsampleAllData", &Class::setDownsampleAllData, "downsample"_a);
    cls.def("getLeafSize", &Class::getLeafSize);
    cls.def("getDownsampleAllData", &Class::getDownsampleAllData);
        
}

template <typename PointT>
void defineFiltersxNdCopyEigenPointFunctor(py::module &m, std::string const & suffix) {
    using Class = pcl::xNdCopyEigenPointFunctor<PointT>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<Eigen::VectorXf, PointT>(), "p1"_a, "p2"_a);
    // Operators not implemented (operator());
        
}

template <typename PointT>
void defineFiltersxNdCopyPointEigenFunctor(py::module &m, std::string const & suffix) {
    using Class = pcl::xNdCopyPointEigenFunctor<PointT>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointT, Eigen::VectorXf>(), "p1"_a, "p2"_a);
    // Operators not implemented (operator());
        
}

void defineFiltersApproximateVoxelGridFunctions(py::module &m) {
}

void defineFiltersApproximateVoxelGridClasses(py::module &sub_module) {
    py::module sub_module_ApproximateVoxelGrid = sub_module.def_submodule("ApproximateVoxelGrid", "Submodule ApproximateVoxelGrid");
    defineFiltersApproximateVoxelGrid<pcl::InterestPoint>(sub_module_ApproximateVoxelGrid, "InterestPoint");
    defineFiltersApproximateVoxelGrid<pcl::PointDEM>(sub_module_ApproximateVoxelGrid, "PointDEM");
    defineFiltersApproximateVoxelGrid<pcl::PointNormal>(sub_module_ApproximateVoxelGrid, "PointNormal");
    defineFiltersApproximateVoxelGrid<pcl::PointSurfel>(sub_module_ApproximateVoxelGrid, "PointSurfel");
    defineFiltersApproximateVoxelGrid<pcl::PointWithRange>(sub_module_ApproximateVoxelGrid, "PointWithRange");
    defineFiltersApproximateVoxelGrid<pcl::PointWithScale>(sub_module_ApproximateVoxelGrid, "PointWithScale");
    defineFiltersApproximateVoxelGrid<pcl::PointWithViewpoint>(sub_module_ApproximateVoxelGrid, "PointWithViewpoint");
    defineFiltersApproximateVoxelGrid<pcl::PointXYZ>(sub_module_ApproximateVoxelGrid, "PointXYZ");
    defineFiltersApproximateVoxelGrid<pcl::PointXYZHSV>(sub_module_ApproximateVoxelGrid, "PointXYZHSV");
    defineFiltersApproximateVoxelGrid<pcl::PointXYZI>(sub_module_ApproximateVoxelGrid, "PointXYZI");
    defineFiltersApproximateVoxelGrid<pcl::PointXYZINormal>(sub_module_ApproximateVoxelGrid, "PointXYZINormal");
    defineFiltersApproximateVoxelGrid<pcl::PointXYZL>(sub_module_ApproximateVoxelGrid, "PointXYZL");
    defineFiltersApproximateVoxelGrid<pcl::PointXYZLNormal>(sub_module_ApproximateVoxelGrid, "PointXYZLNormal");
    defineFiltersApproximateVoxelGrid<pcl::PointXYZRGB>(sub_module_ApproximateVoxelGrid, "PointXYZRGB");
    defineFiltersApproximateVoxelGrid<pcl::PointXYZRGBA>(sub_module_ApproximateVoxelGrid, "PointXYZRGBA");
    defineFiltersApproximateVoxelGrid<pcl::PointXYZRGBL>(sub_module_ApproximateVoxelGrid, "PointXYZRGBL");
    defineFiltersApproximateVoxelGrid<pcl::PointXYZRGBNormal>(sub_module_ApproximateVoxelGrid, "PointXYZRGBNormal");
}