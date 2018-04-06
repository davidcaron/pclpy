
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/mlesac.h>



template <typename PointT>
void defineSampleConsensusMaximumLikelihoodSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = MaximumLikelihoodSampleConsensus<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def_property("em_iterations", &Class::getEMIterations, &Class::setEMIterations);
    cls.def("compute_model", &Class::computeModel);
        
}

void defineSampleConsensusMlesacClasses(py::module &sub_module) {
    py::module sub_module_MaximumLikelihoodSampleConsensus = sub_module.def_submodule("MaximumLikelihoodSampleConsensus", "Submodule MaximumLikelihoodSampleConsensus");
    defineSampleConsensusMaximumLikelihoodSampleConsensus<PointXYZ>(sub_module_MaximumLikelihoodSampleConsensus, "PointXYZ");
    defineSampleConsensusMaximumLikelihoodSampleConsensus<PointXYZI>(sub_module_MaximumLikelihoodSampleConsensus, "PointXYZI");
    defineSampleConsensusMaximumLikelihoodSampleConsensus<PointXYZRGB>(sub_module_MaximumLikelihoodSampleConsensus, "PointXYZRGB");
    defineSampleConsensusMaximumLikelihoodSampleConsensus<PointXYZRGBA>(sub_module_MaximumLikelihoodSampleConsensus, "PointXYZRGBA");
    defineSampleConsensusMaximumLikelihoodSampleConsensus<PointXYZRGBNormal>(sub_module_MaximumLikelihoodSampleConsensus, "PointXYZRGBNormal");
}