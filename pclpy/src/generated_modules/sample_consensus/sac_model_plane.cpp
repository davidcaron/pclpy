
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/sample_consensus/sac_model_plane.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelPlane(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelPlane<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, pcl::SampleConsensusModel<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointCloudConstPtr, bool>(), "cloud"_a, "random"_a=false);
    cls.def(py::init<PointCloudConstPtr, std::vector<int>, bool>(), "cloud"_a, "indices"_a, "random"_a=false);
    cls.def("computeModelCoefficients", &Class::computeModelCoefficients, "samples"_a, "model_coefficients"_a);
    cls.def("selectWithinDistance", &Class::selectWithinDistance, "model_coefficients"_a, "threshold"_a, "inliers"_a);
    cls.def("countWithinDistance", &Class::countWithinDistance, "model_coefficients"_a, "threshold"_a);
    cls.def("optimizeModelCoefficients", &Class::optimizeModelCoefficients, "inliers"_a, "model_coefficients"_a, "optimized_coefficients"_a);
    cls.def("projectPoints", &Class::projectPoints, "inliers"_a, "model_coefficients"_a, "projected_points"_a, "copy_data_fields"_a=true);
    cls.def("doSamplesVerifyModel", &Class::doSamplesVerifyModel, "indices"_a, "model_coefficients"_a, "threshold"_a);
    cls.def("getDistancesToModel", &Class::getDistancesToModel, "model_coefficients"_a, "distances"_a);
    cls.def("getModelType", &Class::getModelType);
        
}

void defineSampleConsensusSacModelPlaneFunctions(py::module &m) {
}

void defineSampleConsensusSacModelPlaneClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelPlane = sub_module.def_submodule("SampleConsensusModelPlane", "Submodule SampleConsensusModelPlane");
    defineSampleConsensusSampleConsensusModelPlane<pcl::PointXYZ>(sub_module_SampleConsensusModelPlane, "PointXYZ");
    defineSampleConsensusSampleConsensusModelPlane<pcl::PointXYZI>(sub_module_SampleConsensusModelPlane, "PointXYZI");
    defineSampleConsensusSampleConsensusModelPlane<pcl::PointXYZRGB>(sub_module_SampleConsensusModelPlane, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelPlane<pcl::PointXYZRGBA>(sub_module_SampleConsensusModelPlane, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModelPlane<pcl::PointXYZRGBNormal>(sub_module_SampleConsensusModelPlane, "PointXYZRGBNormal");
    defineSampleConsensusSacModelPlaneFunctions(sub_module);
}