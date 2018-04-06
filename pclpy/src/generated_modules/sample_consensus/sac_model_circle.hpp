
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_circle.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelCircle2D(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelCircle2D<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, SampleConsensusModel<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<Class::PointCloudConstPtr, bool>(), "cloud"_a, "random"_a=false);
    cls.def(py::init<Class::PointCloudConstPtr, std::vector<int>, bool>(), "cloud"_a, "indices"_a, "random"_a=false);
    // Operators not implemented (operator=);
    cls.def("compute_model_coefficients", &Class::computeModelCoefficients);
    cls.def("select_within_distance", &Class::selectWithinDistance);
    cls.def("count_within_distance", &Class::countWithinDistance);
    cls.def("optimize_model_coefficients", &Class::optimizeModelCoefficients);
    cls.def("project_points", &Class::projectPoints);
    cls.def("do_samples_verify_model", &Class::doSamplesVerifyModel);
        
}

void defineSampleConsensusSacModelCircleClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelCircle2D = sub_module.def_submodule("SampleConsensusModelCircle2D", "Submodule SampleConsensusModelCircle2D");
    defineSampleConsensusSampleConsensusModelCircle2D<PointXYZ>(sub_module_SampleConsensusModelCircle2D, "PointXYZ");
    defineSampleConsensusSampleConsensusModelCircle2D<PointXYZI>(sub_module_SampleConsensusModelCircle2D, "PointXYZI");
    defineSampleConsensusSampleConsensusModelCircle2D<PointXYZRGB>(sub_module_SampleConsensusModelCircle2D, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModelCircle2D<PointXYZRGBA>(sub_module_SampleConsensusModelCircle2D, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModelCircle2D<PointXYZRGBNormal>(sub_module_SampleConsensusModelCircle2D, "PointXYZRGBNormal");
}