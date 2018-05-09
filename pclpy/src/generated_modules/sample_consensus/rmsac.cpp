
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/sample_consensus/rmsac.h>



template <typename PointT>
void defineSampleConsensusRandomizedMEstimatorSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = pcl::RandomizedMEstimatorSampleConsensus<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<pcl::SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<pcl::SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def("computeModel", &Class::computeModel, "debug_verbosity_level"_a=0);
    cls.def("setFractionNrPretest", &Class::setFractionNrPretest, "nr_pretest"_a);
    cls.def("getFractionNrPretest", &Class::getFractionNrPretest);
        
}

void defineSampleConsensusRmsacFunctions(py::module &m) {
}

void defineSampleConsensusRmsacClasses(py::module &sub_module) {
    py::module sub_module_RandomizedMEstimatorSampleConsensus = sub_module.def_submodule("RandomizedMEstimatorSampleConsensus", "Submodule RandomizedMEstimatorSampleConsensus");
    defineSampleConsensusRandomizedMEstimatorSampleConsensus<pcl::PointXYZ>(sub_module_RandomizedMEstimatorSampleConsensus, "PointXYZ");
    defineSampleConsensusRandomizedMEstimatorSampleConsensus<pcl::PointXYZI>(sub_module_RandomizedMEstimatorSampleConsensus, "PointXYZI");
    defineSampleConsensusRandomizedMEstimatorSampleConsensus<pcl::PointXYZRGB>(sub_module_RandomizedMEstimatorSampleConsensus, "PointXYZRGB");
    defineSampleConsensusRandomizedMEstimatorSampleConsensus<pcl::PointXYZRGBA>(sub_module_RandomizedMEstimatorSampleConsensus, "PointXYZRGBA");
    defineSampleConsensusRandomizedMEstimatorSampleConsensus<pcl::PointXYZRGBNormal>(sub_module_RandomizedMEstimatorSampleConsensus, "PointXYZRGBNormal");
}