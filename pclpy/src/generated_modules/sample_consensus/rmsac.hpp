
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/rmsac.h>



template <typename PointT>
void defineSampleConsensusRandomizedMEstimatorSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = RandomizedMEstimatorSampleConsensus<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def_property("fraction_nr_pretest", &Class::getFractionNrPretest, &Class::setFractionNrPretest);
    cls.def("compute_model", &Class::computeModel);
        
}

void defineSampleConsensusRmsacClasses(py::module &sub_module) {
    py::module sub_module_RandomizedMEstimatorSampleConsensus = sub_module.def_submodule("RandomizedMEstimatorSampleConsensus", "Submodule RandomizedMEstimatorSampleConsensus");
    defineSampleConsensusRandomizedMEstimatorSampleConsensus<PointXYZ>(sub_module_RandomizedMEstimatorSampleConsensus, "PointXYZ");
    defineSampleConsensusRandomizedMEstimatorSampleConsensus<PointXYZI>(sub_module_RandomizedMEstimatorSampleConsensus, "PointXYZI");
    defineSampleConsensusRandomizedMEstimatorSampleConsensus<PointXYZRGB>(sub_module_RandomizedMEstimatorSampleConsensus, "PointXYZRGB");
    defineSampleConsensusRandomizedMEstimatorSampleConsensus<PointXYZRGBA>(sub_module_RandomizedMEstimatorSampleConsensus, "PointXYZRGBA");
    defineSampleConsensusRandomizedMEstimatorSampleConsensus<PointXYZRGBNormal>(sub_module_RandomizedMEstimatorSampleConsensus, "PointXYZRGBNormal");
}