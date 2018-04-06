
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>



template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelNormalParallelPlane(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelNormalParallelPlane<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, SampleConsensusModelNormalPlane<PointT,PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("axis", &Class::getAxis, &Class::setAxis);
    cls.def_property("eps_angle", &Class::getEpsAngle, &Class::setEpsAngle);
    cls.def_property("distance_from_origin", &Class::getDistanceFromOrigin, &Class::setDistanceFromOrigin);
    cls.def_property("eps_dist", &Class::getEpsDist, &Class::setEpsDist);
        
}

void defineSampleConsensusSacModelNormalParallelPlaneClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelNormalParallelPlane = sub_module.def_submodule("SampleConsensusModelNormalParallelPlane", "Submodule SampleConsensusModelNormalParallelPlane");
    defineSampleConsensusSampleConsensusModelNormalParallelPlane<PointXYZ, Normal>(sub_module_SampleConsensusModelNormalParallelPlane, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelNormalParallelPlane<PointXYZI, Normal>(sub_module_SampleConsensusModelNormalParallelPlane, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelNormalParallelPlane<PointXYZRGB, Normal>(sub_module_SampleConsensusModelNormalParallelPlane, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelNormalParallelPlane<PointXYZRGBA, Normal>(sub_module_SampleConsensusModelNormalParallelPlane, "PointXYZRGBA_Normal");
}