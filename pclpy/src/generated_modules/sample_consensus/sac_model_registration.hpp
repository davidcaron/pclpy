
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_registration.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelRegistration(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelRegistration<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, SampleConsensusModel<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<Class::PointCloudConstPtr, bool>(), "cloud"_a, "random"_a=false);
    cls.def(py::init<Class::PointCloudConstPtr, std::vector<int>, bool>(), "cloud"_a, "indices"_a, "random"_a=false);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("compute_model_coefficients", &Class::computeModelCoefficients);
    cls.def("select_within_distance", &Class::selectWithinDistance);
    cls.def("count_within_distance", &Class::countWithinDistance);
    cls.def("optimize_model_coefficients", &Class::optimizeModelCoefficients);
    cls.def("project_points", &Class::projectPoints);
    cls.def("do_samples_verify_model", &Class::doSamplesVerifyModel);
    cls.def("set_input_target", py::overload_cast<const PointCloudConstPtr &> (&Class::setInputTarget));
    cls.def("set_input_target", py::overload_cast<const PointCloudConstPtr &, const std::vector<int> &> (&Class::setInputTarget));
        
}

void defineSampleConsensusSacModelRegistrationClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelRegistration = sub_module.def_submodule("SampleConsensusModelRegistration", "Submodule SampleConsensusModelRegistration");
    defineSampleConsensusSampleConsensusModelRegistration<PointXYZ>(sub_module_SampleConsensusModelRegistration, "PointXYZ");
    defineSampleConsensusSampleConsensusModelRegistration<PointXYZI>(sub_module_SampleConsensusModelRegistration, "PointXYZI");
    defineSampleConsensusSampleConsensusModelRegistration<PointXYZRGB>(sub_module_SampleConsensusModelRegistration, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelRegistration<PointXYZRGBA>(sub_module_SampleConsensusModelRegistration, "PointXYZRGBA");
}