
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/ransac.h>



template <typename PointT>
void defineSampleConsensusRandomSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = RandomSampleConsensus<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def("compute_model", &Class::computeModel);
        
}

void defineSampleConsensusRansacClasses(py::module &sub_module) {
    py::module sub_module_RandomSampleConsensus = sub_module.def_submodule("RandomSampleConsensus", "Submodule RandomSampleConsensus");
    defineSampleConsensusRandomSampleConsensus<PointNormal>(sub_module_RandomSampleConsensus, "PointNormal");
    defineSampleConsensusRandomSampleConsensus<PointXYZ>(sub_module_RandomSampleConsensus, "PointXYZ");
    defineSampleConsensusRandomSampleConsensus<PointXYZI>(sub_module_RandomSampleConsensus, "PointXYZI");
    defineSampleConsensusRandomSampleConsensus<PointXYZRGB>(sub_module_RandomSampleConsensus, "PointXYZRGB");
    defineSampleConsensusRandomSampleConsensus<PointXYZRGBA>(sub_module_RandomSampleConsensus, "PointXYZRGBA");
    defineSampleConsensusRandomSampleConsensus<PointXYZRGBNormal>(sub_module_RandomSampleConsensus, "PointXYZRGBNormal");
}