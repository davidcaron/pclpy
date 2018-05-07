
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/sample_consensus/sac.h>



template <typename T>
void defineSampleConsensusSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensus<T>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("computeModel", &Class::computeModel, "debug_verbosity_level"_a=0);
    cls.def("refineModel", &Class::refineModel, "sigma"_a=3.0, "max_iterations"_a=1000);
    cls.def("setSampleConsensusModel", &Class::setSampleConsensusModel, "model"_a);
    cls.def("setDistanceThreshold", &Class::setDistanceThreshold, "threshold"_a);
    cls.def("setMaxIterations", &Class::setMaxIterations, "max_iterations"_a);
    cls.def("setProbability", &Class::setProbability, "probability"_a);
    cls.def("getSampleConsensusModel", &Class::getSampleConsensusModel);
    cls.def("getDistanceThreshold", &Class::getDistanceThreshold);
    cls.def("getMaxIterations", &Class::getMaxIterations);
    cls.def("getProbability", &Class::getProbability);
    cls.def("getRandomSamples", &Class::getRandomSamples, "indices"_a, "nr_samples"_a, "indices_subset"_a);
    cls.def("getModel", &Class::getModel, "model"_a);
    cls.def("getInliers", &Class::getInliers, "inliers"_a);
    cls.def("getModelCoefficients", &Class::getModelCoefficients, "model_coefficients"_a);
        
}

void defineSampleConsensusSacFunctions(py::module &m) {
}

void defineSampleConsensusSacClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensus = sub_module.def_submodule("SampleConsensus", "Submodule SampleConsensus");
    defineSampleConsensusSampleConsensus<pcl::PointNormal>(sub_module_SampleConsensus, "PointNormal");
    defineSampleConsensusSampleConsensus<pcl::PointXYZ>(sub_module_SampleConsensus, "PointXYZ");
    defineSampleConsensusSampleConsensus<pcl::PointXYZI>(sub_module_SampleConsensus, "PointXYZI");
    defineSampleConsensusSampleConsensus<pcl::PointXYZRGB>(sub_module_SampleConsensus, "PointXYZRGB");
    defineSampleConsensusSampleConsensus<pcl::PointXYZRGBA>(sub_module_SampleConsensus, "PointXYZRGBA");
    defineSampleConsensusSampleConsensus<pcl::PointXYZRGBNormal>(sub_module_SampleConsensus, "PointXYZRGBNormal");
}