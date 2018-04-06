
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/approximate_voxel_grid.h>



template <typename PointT>
void defineFiltersApproximateVoxelGrid(py::module &m, std::string const & suffix) {
    using Class = ApproximateVoxelGrid<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("downsample_all_data", &Class::getDownsampleAllData, &Class::setDownsampleAllData);
    // Operators not implemented (operator=);
    cls.def("set_leaf_size", py::overload_cast<const Eigen::Vector3f &> (&Class::setLeafSize));
    cls.def("set_leaf_size", py::overload_cast<float, float, float> (&Class::setLeafSize));
        
}

template <typename PointT>
void defineFiltersxNdCopyEigenPointFunctor(py::module &m, std::string const & suffix) {
    using Class = xNdCopyEigenPointFunctor<PointT>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<Eigen::VectorXf, PointT>(), "p1"_a, "p2"_a);
        
}

template <typename PointT>
void defineFiltersxNdCopyPointEigenFunctor(py::module &m, std::string const & suffix) {
    using Class = xNdCopyPointEigenFunctor<PointT>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointT, Eigen::VectorXf>(), "p1"_a, "p2"_a);
        
}

void defineFiltersApproximateVoxelGridClasses(py::module &sub_module) {
    py::module sub_module_ApproximateVoxelGrid = sub_module.def_submodule("ApproximateVoxelGrid", "Submodule ApproximateVoxelGrid");
    defineFiltersApproximateVoxelGrid<InterestPoint>(sub_module_ApproximateVoxelGrid, "InterestPoint");
    defineFiltersApproximateVoxelGrid<PointDEM>(sub_module_ApproximateVoxelGrid, "PointDEM");
    defineFiltersApproximateVoxelGrid<PointNormal>(sub_module_ApproximateVoxelGrid, "PointNormal");
    defineFiltersApproximateVoxelGrid<PointSurfel>(sub_module_ApproximateVoxelGrid, "PointSurfel");
    defineFiltersApproximateVoxelGrid<PointWithRange>(sub_module_ApproximateVoxelGrid, "PointWithRange");
    defineFiltersApproximateVoxelGrid<PointWithScale>(sub_module_ApproximateVoxelGrid, "PointWithScale");
    defineFiltersApproximateVoxelGrid<PointWithViewpoint>(sub_module_ApproximateVoxelGrid, "PointWithViewpoint");
    defineFiltersApproximateVoxelGrid<PointXYZ>(sub_module_ApproximateVoxelGrid, "PointXYZ");
    defineFiltersApproximateVoxelGrid<PointXYZHSV>(sub_module_ApproximateVoxelGrid, "PointXYZHSV");
    defineFiltersApproximateVoxelGrid<PointXYZI>(sub_module_ApproximateVoxelGrid, "PointXYZI");
    defineFiltersApproximateVoxelGrid<PointXYZINormal>(sub_module_ApproximateVoxelGrid, "PointXYZINormal");
    defineFiltersApproximateVoxelGrid<PointXYZL>(sub_module_ApproximateVoxelGrid, "PointXYZL");
    defineFiltersApproximateVoxelGrid<PointXYZLNormal>(sub_module_ApproximateVoxelGrid, "PointXYZLNormal");
    defineFiltersApproximateVoxelGrid<PointXYZRGB>(sub_module_ApproximateVoxelGrid, "PointXYZRGB");
    defineFiltersApproximateVoxelGrid<PointXYZRGBA>(sub_module_ApproximateVoxelGrid, "PointXYZRGBA");
    defineFiltersApproximateVoxelGrid<PointXYZRGBL>(sub_module_ApproximateVoxelGrid, "PointXYZRGBL");
    defineFiltersApproximateVoxelGrid<PointXYZRGBNormal>(sub_module_ApproximateVoxelGrid, "PointXYZRGBNormal");
}