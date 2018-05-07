
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/sample_consensus/sac_model_cylinder.h>



template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelCylinder(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelCylinder<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, pcl::SampleConsensusModel<PointT>, pcl::SampleConsensusModelFromNormals<PointT, PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointCloudConstPtr, bool>(), "cloud"_a, "random"_a=false);
    cls.def(py::init<PointCloudConstPtr, std::vector<int>, bool>(), "cloud"_a, "indices"_a, "random"_a=false);
    // Operators not implemented (operator=);
    cls.def("computeModelCoefficients", &Class::computeModelCoefficients, "samples"_a, "model_coefficients"_a);
    cls.def("selectWithinDistance", &Class::selectWithinDistance, "model_coefficients"_a, "threshold"_a, "inliers"_a);
    cls.def("countWithinDistance", &Class::countWithinDistance, "model_coefficients"_a, "threshold"_a);
    cls.def("optimizeModelCoefficients", &Class::optimizeModelCoefficients, "inliers"_a, "model_coefficients"_a, "optimized_coefficients"_a);
    cls.def("projectPoints", &Class::projectPoints, "inliers"_a, "model_coefficients"_a, "projected_points"_a, "copy_data_fields"_a=true);
    cls.def("doSamplesVerifyModel", &Class::doSamplesVerifyModel, "indices"_a, "model_coefficients"_a, "threshold"_a);
    cls.def("setEpsAngle", &Class::setEpsAngle, "ea"_a);
    cls.def("setAxis", &Class::setAxis, "ax"_a);
    cls.def("getEpsAngle", &Class::getEpsAngle);
    cls.def("getAxis", &Class::getAxis);
    cls.def("getDistancesToModel", &Class::getDistancesToModel, "model_coefficients"_a, "distances"_a);
    cls.def("getModelType", &Class::getModelType);
        
}

void defineSampleConsensusSacModelCylinderFunctions(py::module &m) {
}

void defineSampleConsensusSacModelCylinderClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelCylinder = sub_module.def_submodule("SampleConsensusModelCylinder", "Submodule SampleConsensusModelCylinder");
    defineSampleConsensusSampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>(sub_module_SampleConsensusModelCylinder, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelCylinder<pcl::PointXYZI, pcl::Normal>(sub_module_SampleConsensusModelCylinder, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelCylinder<pcl::PointXYZRGB, pcl::Normal>(sub_module_SampleConsensusModelCylinder, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelCylinder<pcl::PointXYZRGBA, pcl::Normal>(sub_module_SampleConsensusModelCylinder, "PointXYZRGBA_Normal");
}