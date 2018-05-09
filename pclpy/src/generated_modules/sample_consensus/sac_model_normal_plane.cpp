
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/sample_consensus/sac_model_normal_plane.h>



template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelNormalPlane(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelNormalPlane<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, pcl::SampleConsensusModelPlane<PointT>, pcl::SampleConsensusModelFromNormals<PointT, PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("selectWithinDistance", &Class::selectWithinDistance, "model_coefficients"_a, "threshold"_a, "inliers"_a);
    cls.def("countWithinDistance", &Class::countWithinDistance, "model_coefficients"_a, "threshold"_a);
    cls.def("getDistancesToModel", &Class::getDistancesToModel, "model_coefficients"_a, "distances"_a);
    cls.def("getModelType", &Class::getModelType);
        
}

void defineSampleConsensusSacModelNormalPlaneFunctions(py::module &m) {
}

void defineSampleConsensusSacModelNormalPlaneClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelNormalPlane = sub_module.def_submodule("SampleConsensusModelNormalPlane", "Submodule SampleConsensusModelNormalPlane");
    defineSampleConsensusSampleConsensusModelNormalPlane<pcl::PointXYZ, pcl::Normal>(sub_module_SampleConsensusModelNormalPlane, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelNormalPlane<pcl::PointXYZI, pcl::Normal>(sub_module_SampleConsensusModelNormalPlane, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelNormalPlane<pcl::PointXYZRGB, pcl::Normal>(sub_module_SampleConsensusModelNormalPlane, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelNormalPlane<pcl::PointXYZRGBA, pcl::Normal>(sub_module_SampleConsensusModelNormalPlane, "PointXYZRGBA_Normal");
}