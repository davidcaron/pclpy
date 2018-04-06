
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac.h>



template <typename T>
void defineSampleConsensusSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = SampleConsensus<T>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("sample_consensus_model", &Class::getSampleConsensusModel, &Class::setSampleConsensusModel);
    cls.def_property("distance_threshold", &Class::getDistanceThreshold, &Class::setDistanceThreshold);
    cls.def_property("max_iterations", &Class::getMaxIterations, &Class::setMaxIterations);
    cls.def_property("probability", &Class::getProbability, &Class::setProbability);
    cls.def("compute_model", &Class::computeModel);
    cls.def("refine_model", &Class::refineModel);
        
}

void defineSampleConsensusSacClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensus = sub_module.def_submodule("SampleConsensus", "Submodule SampleConsensus");
    defineSampleConsensusSampleConsensus<PointNormal>(sub_module_SampleConsensus, "PointNormal");
    defineSampleConsensusSampleConsensus<PointXYZ>(sub_module_SampleConsensus, "PointXYZ");
    defineSampleConsensusSampleConsensus<PointXYZI>(sub_module_SampleConsensus, "PointXYZI");
    defineSampleConsensusSampleConsensus<PointXYZRGB>(sub_module_SampleConsensus, "PointXYZRGB");
    defineSampleConsensusSampleConsensus<PointXYZRGBA>(sub_module_SampleConsensus, "PointXYZRGBA");
    defineSampleConsensusSampleConsensus<PointXYZRGBNormal>(sub_module_SampleConsensus, "PointXYZRGBNormal");
}