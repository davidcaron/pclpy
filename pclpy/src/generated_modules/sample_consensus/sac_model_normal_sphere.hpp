
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/sample_consensus/sac_model_normal_sphere.h>



template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelNormalSphere(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelNormalSphere<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, pcl::SampleConsensusModelSphere<PointT>, pcl::SampleConsensusModelFromNormals<PointT, PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("selectWithinDistance", &Class::selectWithinDistance, "model_coefficients"_a, "threshold"_a, "inliers"_a);
    cls.def("countWithinDistance", &Class::countWithinDistance, "model_coefficients"_a, "threshold"_a);
    cls.def("getDistancesToModel", &Class::getDistancesToModel, "model_coefficients"_a, "distances"_a);
    cls.def("getModelType", &Class::getModelType);
        
}

void defineSampleConsensusSacModelNormalSphereFunctions(py::module &m) {
}

void defineSampleConsensusSacModelNormalSphereClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelNormalSphere = sub_module.def_submodule("SampleConsensusModelNormalSphere", "Submodule SampleConsensusModelNormalSphere");
    defineSampleConsensusSampleConsensusModelNormalSphere<pcl::PointXYZ, pcl::Normal>(sub_module_SampleConsensusModelNormalSphere, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelNormalSphere<pcl::PointXYZI, pcl::Normal>(sub_module_SampleConsensusModelNormalSphere, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelNormalSphere<pcl::PointXYZRGB, pcl::Normal>(sub_module_SampleConsensusModelNormalSphere, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelNormalSphere<pcl::PointXYZRGBA, pcl::Normal>(sub_module_SampleConsensusModelNormalSphere, "PointXYZRGBA_Normal");
}