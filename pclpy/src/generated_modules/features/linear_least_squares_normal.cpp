
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/linear_least_squares_normal.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesLinearLeastSquaresNormalEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::LinearLeastSquaresNormalEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computePointNormal", py::overload_cast<const int, const int, PointOutT &> (&Class::computePointNormal), "pos_x"_a, "pos_y"_a, "normal"_a);
    cls.def("setNormalSmoothingSize", &Class::setNormalSmoothingSize, "normal_smoothing_size"_a);
    cls.def("setDepthDependentSmoothing", &Class::setDepthDependentSmoothing, "use_depth_dependent_smoothing"_a);
    cls.def("setMaxDepthChangeFactor", &Class::setMaxDepthChangeFactor, "max_depth_change_factor"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
        
}

void defineFeaturesLinearLeastSquaresNormalFunctions(py::module &m) {
}

void defineFeaturesLinearLeastSquaresNormalClasses(py::module &sub_module) {
    py::module sub_module_LinearLeastSquaresNormalEstimation = sub_module.def_submodule("LinearLeastSquaresNormalEstimation", "Submodule LinearLeastSquaresNormalEstimation");
    defineFeaturesLinearLeastSquaresNormalEstimation<pcl::PointXYZ, pcl::Normal>(sub_module_LinearLeastSquaresNormalEstimation, "PointXYZ_Normal");
    defineFeaturesLinearLeastSquaresNormalEstimation<pcl::PointXYZI, pcl::Normal>(sub_module_LinearLeastSquaresNormalEstimation, "PointXYZI_Normal");
    defineFeaturesLinearLeastSquaresNormalEstimation<pcl::PointXYZRGB, pcl::Normal>(sub_module_LinearLeastSquaresNormalEstimation, "PointXYZRGB_Normal");
    defineFeaturesLinearLeastSquaresNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>(sub_module_LinearLeastSquaresNormalEstimation, "PointXYZRGBA_Normal");
}