
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelPerpendicularPlane(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelPerpendicularPlane<PointT>;
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

void defineSampleConsensusSacModelPerpendicularPlaneFunctions(py::module &m) {
}

void defineSampleConsensusSacModelPerpendicularPlaneClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelPerpendicularPlane = sub_module.def_submodule("SampleConsensusModelPerpendicularPlane", "Submodule SampleConsensusModelPerpendicularPlane");
    defineSampleConsensusSampleConsensusModelPerpendicularPlane<pcl::PointXYZ>(sub_module_SampleConsensusModelPerpendicularPlane, "PointXYZ");
    defineSampleConsensusSampleConsensusModelPerpendicularPlane<pcl::PointXYZI>(sub_module_SampleConsensusModelPerpendicularPlane, "PointXYZI");
    defineSampleConsensusSampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>(sub_module_SampleConsensusModelPerpendicularPlane, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelPerpendicularPlane<pcl::PointXYZRGBA>(sub_module_SampleConsensusModelPerpendicularPlane, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModelPerpendicularPlane<pcl::PointXYZRGBNormal>(sub_module_SampleConsensusModelPerpendicularPlane, "PointXYZRGBNormal");
}