
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelPerpendicularPlane(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelPerpendicularPlane<PointT>;
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

void defineSampleConsensusSacModelPerpendicularPlaneClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelPerpendicularPlane = sub_module.def_submodule("SampleConsensusModelPerpendicularPlane", "Submodule SampleConsensusModelPerpendicularPlane");
    defineSampleConsensusSampleConsensusModelPerpendicularPlane<PointXYZ>(sub_module_SampleConsensusModelPerpendicularPlane, "PointXYZ");
    defineSampleConsensusSampleConsensusModelPerpendicularPlane<PointXYZI>(sub_module_SampleConsensusModelPerpendicularPlane, "PointXYZI");
    defineSampleConsensusSampleConsensusModelPerpendicularPlane<PointXYZRGB>(sub_module_SampleConsensusModelPerpendicularPlane, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelPerpendicularPlane<PointXYZRGBA>(sub_module_SampleConsensusModelPerpendicularPlane, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModelPerpendicularPlane<PointXYZRGBNormal>(sub_module_SampleConsensusModelPerpendicularPlane, "PointXYZRGBNormal");
}