
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_parallel_plane.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelParallelPlane(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelParallelPlane<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, SampleConsensusModelPlane<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("axis", &Class::getAxis, &Class::setAxis);
    cls.def_property("eps_angle", &Class::getEpsAngle, &Class::setEpsAngle);
    cls.def("select_within_distance", &Class::selectWithinDistance);
    cls.def("count_within_distance", &Class::countWithinDistance);
        
}

void defineSampleConsensusSacModelParallelPlaneClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelParallelPlane = sub_module.def_submodule("SampleConsensusModelParallelPlane", "Submodule SampleConsensusModelParallelPlane");
    defineSampleConsensusSampleConsensusModelParallelPlane<PointXYZ>(sub_module_SampleConsensusModelParallelPlane, "PointXYZ");
    defineSampleConsensusSampleConsensusModelParallelPlane<PointXYZI>(sub_module_SampleConsensusModelParallelPlane, "PointXYZI");
    defineSampleConsensusSampleConsensusModelParallelPlane<PointXYZRGB>(sub_module_SampleConsensusModelParallelPlane, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelParallelPlane<PointXYZRGBA>(sub_module_SampleConsensusModelParallelPlane, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModelParallelPlane<PointXYZRGBNormal>(sub_module_SampleConsensusModelParallelPlane, "PointXYZRGBNormal");
}