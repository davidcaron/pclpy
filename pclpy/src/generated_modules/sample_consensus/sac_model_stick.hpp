
#include <pcl/sample_consensus/sac_model_stick.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelStick(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelStick<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, pcl::SampleConsensusModel<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointCloudConstPtr, bool>(), "cloud"_a, "random"_a=false);
    cls.def(py::init<PointCloudConstPtr, std::vector<int>, bool>(), "cloud"_a, "indices"_a, "random"_a=false);
    cls.def("computeModelCoefficients", &Class::computeModelCoefficients, "samples"_a, "model_coefficients"_a);
    cls.def("selectWithinDistance", &Class::selectWithinDistance, "model_coefficients"_a, "threshold"_a, "inliers"_a);
    cls.def("countWithinDistance", &Class::countWithinDistance, "model_coefficients"_a, "threshold"_a);
    cls.def("optimizeModelCoefficients", &Class::optimizeModelCoefficients, "inliers"_a, "model_coefficients"_a, "optimized_coefficients"_a);
    cls.def("projectPoints", &Class::projectPoints, "inliers"_a, "model_coefficients"_a, "projected_points"_a, "copy_data_fields"_a=true);
    cls.def("doSamplesVerifyModel", &Class::doSamplesVerifyModel, "indices"_a, "model_coefficients"_a, "threshold"_a);
    cls.def("getDistancesToModel", &Class::getDistancesToModel, "model_coefficients"_a, "distances"_a);
    cls.def("getModelType", &Class::getModelType);
        
}

void defineSampleConsensusSacModelStickFunctions(py::module &m) {
}

void defineSampleConsensusSacModelStickClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelStick = sub_module.def_submodule("SampleConsensusModelStick", "Submodule SampleConsensusModelStick");
    defineSampleConsensusSampleConsensusModelStick<pcl::PointXYZ>(sub_module_SampleConsensusModelStick, "PointXYZ");
    defineSampleConsensusSampleConsensusModelStick<pcl::PointXYZI>(sub_module_SampleConsensusModelStick, "PointXYZI");
    defineSampleConsensusSampleConsensusModelStick<pcl::PointXYZRGB>(sub_module_SampleConsensusModelStick, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelStick<pcl::PointXYZRGBA>(sub_module_SampleConsensusModelStick, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModelStick<pcl::PointXYZRGBNormal>(sub_module_SampleConsensusModelStick, "PointXYZRGBNormal");
}