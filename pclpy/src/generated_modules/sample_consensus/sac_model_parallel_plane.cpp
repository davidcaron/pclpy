
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/sample_consensus/sac_model_parallel_plane.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelParallelPlane(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelParallelPlane<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, pcl::SampleConsensusModelPlane<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("selectWithinDistance", &Class::selectWithinDistance, "model_coefficients"_a, "threshold"_a, "inliers"_a);
    cls.def("countWithinDistance", &Class::countWithinDistance, "model_coefficients"_a, "threshold"_a);
    cls.def("setAxis", &Class::setAxis, "ax"_a);
    cls.def("setEpsAngle", &Class::setEpsAngle, "ea"_a);
    cls.def("getAxis", &Class::getAxis);
    cls.def("getEpsAngle", &Class::getEpsAngle);
    cls.def("getDistancesToModel", &Class::getDistancesToModel, "model_coefficients"_a, "distances"_a);
    cls.def("getModelType", &Class::getModelType);
        
}

void defineSampleConsensusSacModelParallelPlaneFunctions(py::module &m) {
}

void defineSampleConsensusSacModelParallelPlaneClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelParallelPlane = sub_module.def_submodule("SampleConsensusModelParallelPlane", "Submodule SampleConsensusModelParallelPlane");
    defineSampleConsensusSampleConsensusModelParallelPlane<pcl::PointXYZ>(sub_module_SampleConsensusModelParallelPlane, "PointXYZ");
    defineSampleConsensusSampleConsensusModelParallelPlane<pcl::PointXYZI>(sub_module_SampleConsensusModelParallelPlane, "PointXYZI");
    defineSampleConsensusSampleConsensusModelParallelPlane<pcl::PointXYZRGB>(sub_module_SampleConsensusModelParallelPlane, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelParallelPlane<pcl::PointXYZRGBA>(sub_module_SampleConsensusModelParallelPlane, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModelParallelPlane<pcl::PointXYZRGBNormal>(sub_module_SampleConsensusModelParallelPlane, "PointXYZRGBNormal");
}