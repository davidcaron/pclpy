
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/linear_least_squares_normal.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesLinearLeastSquaresNormalEstimation(py::module &m, std::string const & suffix) {
    using Class = LinearLeastSquaresNormalEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_normal_smoothing_size", &Class::setNormalSmoothingSize);
    cls.def("set_depth_dependent_smoothing", &Class::setDepthDependentSmoothing);
    cls.def("set_max_depth_change_factor", &Class::setMaxDepthChangeFactor);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("compute_point_normal", py::overload_cast<const int, const int, PointOutT &> (&Class::computePointNormal));
        
}

void defineFeaturesLinearLeastSquaresNormalClasses(py::module &sub_module) {
    py::module sub_module_LinearLeastSquaresNormalEstimation = sub_module.def_submodule("LinearLeastSquaresNormalEstimation", "Submodule LinearLeastSquaresNormalEstimation");
    defineFeaturesLinearLeastSquaresNormalEstimation<PointXYZ, Normal>(sub_module_LinearLeastSquaresNormalEstimation, "PointXYZ_Normal");
    defineFeaturesLinearLeastSquaresNormalEstimation<PointXYZI, Normal>(sub_module_LinearLeastSquaresNormalEstimation, "PointXYZI_Normal");
    defineFeaturesLinearLeastSquaresNormalEstimation<PointXYZRGB, Normal>(sub_module_LinearLeastSquaresNormalEstimation, "PointXYZRGB_Normal");
    defineFeaturesLinearLeastSquaresNormalEstimation<PointXYZRGBA, Normal>(sub_module_LinearLeastSquaresNormalEstimation, "PointXYZRGBA_Normal");
}