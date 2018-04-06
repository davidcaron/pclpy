
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_normal_sphere.h>



template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelNormalSphere(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelNormalSphere<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, SampleConsensusModelSphere<PointT>, SampleConsensusModelFromNormals<PointT,PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("select_within_distance", &Class::selectWithinDistance);
    cls.def("count_within_distance", &Class::countWithinDistance);
        
}

void defineSampleConsensusSacModelNormalSphereClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelNormalSphere = sub_module.def_submodule("SampleConsensusModelNormalSphere", "Submodule SampleConsensusModelNormalSphere");
    defineSampleConsensusSampleConsensusModelNormalSphere<PointXYZ, Normal>(sub_module_SampleConsensusModelNormalSphere, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelNormalSphere<PointXYZI, Normal>(sub_module_SampleConsensusModelNormalSphere, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelNormalSphere<PointXYZRGB, Normal>(sub_module_SampleConsensusModelNormalSphere, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelNormalSphere<PointXYZRGBA, Normal>(sub_module_SampleConsensusModelNormalSphere, "PointXYZRGBA_Normal");
}