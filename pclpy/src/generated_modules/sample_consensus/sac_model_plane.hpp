
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_plane.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelPlane(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelPlane<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, SampleConsensusModel<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<Class::PointCloudConstPtr, bool>(), "cloud"_a, "random"_a=false);
    cls.def(py::init<Class::PointCloudConstPtr, std::vector<int>, bool>(), "cloud"_a, "indices"_a, "random"_a=false);
    cls.def("compute_model_coefficients", &Class::computeModelCoefficients);
    cls.def("select_within_distance", &Class::selectWithinDistance);
    cls.def("count_within_distance", &Class::countWithinDistance);
    cls.def("optimize_model_coefficients", &Class::optimizeModelCoefficients);
    cls.def("project_points", &Class::projectPoints);
    cls.def("do_samples_verify_model", &Class::doSamplesVerifyModel);
        
}

void defineSampleConsensusSacModelPlaneClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelPlane = sub_module.def_submodule("SampleConsensusModelPlane", "Submodule SampleConsensusModelPlane");
    defineSampleConsensusSampleConsensusModelPlane<PointXYZ>(sub_module_SampleConsensusModelPlane, "PointXYZ");
    defineSampleConsensusSampleConsensusModelPlane<PointXYZI>(sub_module_SampleConsensusModelPlane, "PointXYZI");
    defineSampleConsensusSampleConsensusModelPlane<PointXYZRGB>(sub_module_SampleConsensusModelPlane, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelPlane<PointXYZRGBA>(sub_module_SampleConsensusModelPlane, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModelPlane<PointXYZRGBNormal>(sub_module_SampleConsensusModelPlane, "PointXYZRGBNormal");
}