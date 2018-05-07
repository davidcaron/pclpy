
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/sample_consensus/rransac.h>



template <typename PointT>
void defineSampleConsensusRandomizedRandomSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = pcl::RandomizedRandomSampleConsensus<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<pcl::SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<pcl::SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def("computeModel", &Class::computeModel, "debug_verbosity_level"_a=0);
    cls.def("setFractionNrPretest", &Class::setFractionNrPretest, "nr_pretest"_a);
    cls.def("getFractionNrPretest", &Class::getFractionNrPretest);
        
}

void defineSampleConsensusRransacFunctions(py::module &m) {
}

void defineSampleConsensusRransacClasses(py::module &sub_module) {
    py::module sub_module_RandomizedRandomSampleConsensus = sub_module.def_submodule("RandomizedRandomSampleConsensus", "Submodule RandomizedRandomSampleConsensus");
    defineSampleConsensusRandomizedRandomSampleConsensus<pcl::PointXYZ>(sub_module_RandomizedRandomSampleConsensus, "PointXYZ");
    defineSampleConsensusRandomizedRandomSampleConsensus<pcl::PointXYZI>(sub_module_RandomizedRandomSampleConsensus, "PointXYZI");
    defineSampleConsensusRandomizedRandomSampleConsensus<pcl::PointXYZRGB>(sub_module_RandomizedRandomSampleConsensus, "PointXYZRGB");
    defineSampleConsensusRandomizedRandomSampleConsensus<pcl::PointXYZRGBA>(sub_module_RandomizedRandomSampleConsensus, "PointXYZRGBA");
    defineSampleConsensusRandomizedRandomSampleConsensus<pcl::PointXYZRGBNormal>(sub_module_RandomizedRandomSampleConsensus, "PointXYZRGBNormal");
}