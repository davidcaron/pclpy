
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>



template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelNormalParallelPlane(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelNormalParallelPlane<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, pcl::SampleConsensusModelNormalPlane<PointT, PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("setAxis", &Class::setAxis, "ax"_a);
    cls.def("setEpsAngle", &Class::setEpsAngle, "ea"_a);
    cls.def("setDistanceFromOrigin", &Class::setDistanceFromOrigin, "d"_a);
    cls.def("setEpsDist", &Class::setEpsDist, "delta"_a);
    cls.def("getAxis", &Class::getAxis);
    cls.def("getEpsAngle", &Class::getEpsAngle);
    cls.def("getDistanceFromOrigin", &Class::getDistanceFromOrigin);
    cls.def("getEpsDist", &Class::getEpsDist);
    cls.def("getModelType", &Class::getModelType);
        
}

void defineSampleConsensusSacModelNormalParallelPlaneFunctions(py::module &m) {
}

void defineSampleConsensusSacModelNormalParallelPlaneClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelNormalParallelPlane = sub_module.def_submodule("SampleConsensusModelNormalParallelPlane", "Submodule SampleConsensusModelNormalParallelPlane");
    defineSampleConsensusSampleConsensusModelNormalParallelPlane<pcl::PointXYZ, pcl::Normal>(sub_module_SampleConsensusModelNormalParallelPlane, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelNormalParallelPlane<pcl::PointXYZI, pcl::Normal>(sub_module_SampleConsensusModelNormalParallelPlane, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelNormalParallelPlane<pcl::PointXYZRGB, pcl::Normal>(sub_module_SampleConsensusModelNormalParallelPlane, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelNormalParallelPlane<pcl::PointXYZRGBA, pcl::Normal>(sub_module_SampleConsensusModelNormalParallelPlane, "PointXYZRGBA_Normal");
}