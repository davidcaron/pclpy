
#include <pcl/sample_consensus/sac_model_parallel_line.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelParallelLine(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelParallelLine<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, pcl::SampleConsensusModelLine<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("selectWithinDistance", &Class::selectWithinDistance, "model_coefficients"_a, "threshold"_a, "inliers"_a);
    cls.def("countWithinDistance", &Class::countWithinDistance, "model_coefficients"_a, "threshold"_a);
    cls.def("setAxis", &Class::setAxis, "ax"_a);
    cls.def("setEpsAngle", &Class::setEpsAngle, "ea"_a);
    cls.def("getAxis", &Class::getAxis);
    cls.def("getEpsAngle", &Class::getEpsAngle);
    cls.def("getDistancesToModel", &Class::getDistancesToModel, "model_coefficients"_a, "distances"_a);
    cls.def("getModelType", &Class::getModelType);
        
}

void defineSampleConsensusSacModelParallelLineFunctions(py::module &m) {
}

void defineSampleConsensusSacModelParallelLineClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelParallelLine = sub_module.def_submodule("SampleConsensusModelParallelLine", "Submodule SampleConsensusModelParallelLine");
    defineSampleConsensusSampleConsensusModelParallelLine<pcl::PointXYZ>(sub_module_SampleConsensusModelParallelLine, "PointXYZ");
    defineSampleConsensusSampleConsensusModelParallelLine<pcl::PointXYZI>(sub_module_SampleConsensusModelParallelLine, "PointXYZI");
    defineSampleConsensusSampleConsensusModelParallelLine<pcl::PointXYZRGB>(sub_module_SampleConsensusModelParallelLine, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelParallelLine<pcl::PointXYZRGBA>(sub_module_SampleConsensusModelParallelLine, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModelParallelLine<pcl::PointXYZRGBNormal>(sub_module_SampleConsensusModelParallelLine, "PointXYZRGBNormal");
}