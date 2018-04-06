
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/msac.h>



template <typename PointT>
void defineSampleConsensusMEstimatorSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = MEstimatorSampleConsensus<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def("compute_model", &Class::computeModel);
        
}

void defineSampleConsensusMsacClasses(py::module &sub_module) {
    py::module sub_module_MEstimatorSampleConsensus = sub_module.def_submodule("MEstimatorSampleConsensus", "Submodule MEstimatorSampleConsensus");
    defineSampleConsensusMEstimatorSampleConsensus<PointXYZ>(sub_module_MEstimatorSampleConsensus, "PointXYZ");
    defineSampleConsensusMEstimatorSampleConsensus<PointXYZI>(sub_module_MEstimatorSampleConsensus, "PointXYZI");
    defineSampleConsensusMEstimatorSampleConsensus<PointXYZRGB>(sub_module_MEstimatorSampleConsensus, "PointXYZRGB");
    defineSampleConsensusMEstimatorSampleConsensus<PointXYZRGBA>(sub_module_MEstimatorSampleConsensus, "PointXYZRGBA");
    defineSampleConsensusMEstimatorSampleConsensus<PointXYZRGBNormal>(sub_module_MEstimatorSampleConsensus, "PointXYZRGBNormal");
}