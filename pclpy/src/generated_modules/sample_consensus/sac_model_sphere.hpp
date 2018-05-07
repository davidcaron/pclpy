
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/sample_consensus/sac_model_sphere.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelSphere(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelSphere<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, pcl::SampleConsensusModel<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointCloudConstPtr, bool>(), "cloud"_a, "random"_a=false);
    cls.def(py::init<PointCloudConstPtr, std::vector<int>, bool>(), "cloud"_a, "indices"_a, "random"_a=false);
    // Operators not implemented (operator=);
    cls.def("computeModelCoefficients", &Class::computeModelCoefficients, "samples"_a, "model_coefficients"_a);
    cls.def("selectWithinDistance", &Class::selectWithinDistance, "model_coefficients"_a, "threshold"_a, "inliers"_a);
    cls.def("countWithinDistance", &Class::countWithinDistance, "model_coefficients"_a, "threshold"_a);
    cls.def("optimizeModelCoefficients", &Class::optimizeModelCoefficients, "inliers"_a, "model_coefficients"_a, "optimized_coefficients"_a);
    cls.def("projectPoints", &Class::projectPoints, "inliers"_a, "model_coefficients"_a, "projected_points"_a, "copy_data_fields"_a=true);
    cls.def("doSamplesVerifyModel", &Class::doSamplesVerifyModel, "indices"_a, "model_coefficients"_a, "threshold"_a);
    cls.def("getDistancesToModel", &Class::getDistancesToModel, "model_coefficients"_a, "distances"_a);
    cls.def("getModelType", &Class::getModelType);
        
}

void defineSampleConsensusSacModelSphereFunctions(py::module &m) {
}

void defineSampleConsensusSacModelSphereClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelSphere = sub_module.def_submodule("SampleConsensusModelSphere", "Submodule SampleConsensusModelSphere");
    defineSampleConsensusSampleConsensusModelSphere<pcl::PointXYZ>(sub_module_SampleConsensusModelSphere, "PointXYZ");
    defineSampleConsensusSampleConsensusModelSphere<pcl::PointXYZI>(sub_module_SampleConsensusModelSphere, "PointXYZI");
    defineSampleConsensusSampleConsensusModelSphere<pcl::PointXYZRGB>(sub_module_SampleConsensusModelSphere, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelSphere<pcl::PointXYZRGBA>(sub_module_SampleConsensusModelSphere, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModelSphere<pcl::PointXYZRGBNormal>(sub_module_SampleConsensusModelSphere, "PointXYZRGBNormal");
}