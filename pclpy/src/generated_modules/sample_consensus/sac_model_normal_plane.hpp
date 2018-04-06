
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_normal_plane.h>



template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelNormalPlane(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelNormalPlane<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, SampleConsensusModelPlane<PointT>, SampleConsensusModelFromNormals<PointT,PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("select_within_distance", &Class::selectWithinDistance);
    cls.def("count_within_distance", &Class::countWithinDistance);
        
}

void defineSampleConsensusSacModelNormalPlaneClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelNormalPlane = sub_module.def_submodule("SampleConsensusModelNormalPlane", "Submodule SampleConsensusModelNormalPlane");
    defineSampleConsensusSampleConsensusModelNormalPlane<PointXYZ, Normal>(sub_module_SampleConsensusModelNormalPlane, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelNormalPlane<PointXYZI, Normal>(sub_module_SampleConsensusModelNormalPlane, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelNormalPlane<PointXYZRGB, Normal>(sub_module_SampleConsensusModelNormalPlane, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelNormalPlane<PointXYZRGBA, Normal>(sub_module_SampleConsensusModelNormalPlane, "PointXYZRGBA_Normal");
}