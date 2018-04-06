
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/rransac.h>



template <typename PointT>
void defineSampleConsensusRandomizedRandomSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = RandomizedRandomSampleConsensus<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def_property("fraction_nr_pretest", &Class::getFractionNrPretest, &Class::setFractionNrPretest);
    cls.def("compute_model", &Class::computeModel);
        
}

void defineSampleConsensusRransacClasses(py::module &sub_module) {
    py::module sub_module_RandomizedRandomSampleConsensus = sub_module.def_submodule("RandomizedRandomSampleConsensus", "Submodule RandomizedRandomSampleConsensus");
    defineSampleConsensusRandomizedRandomSampleConsensus<PointXYZ>(sub_module_RandomizedRandomSampleConsensus, "PointXYZ");
    defineSampleConsensusRandomizedRandomSampleConsensus<PointXYZI>(sub_module_RandomizedRandomSampleConsensus, "PointXYZI");
    defineSampleConsensusRandomizedRandomSampleConsensus<PointXYZRGB>(sub_module_RandomizedRandomSampleConsensus, "PointXYZRGB");
    defineSampleConsensusRandomizedRandomSampleConsensus<PointXYZRGBA>(sub_module_RandomizedRandomSampleConsensus, "PointXYZRGBA");
    defineSampleConsensusRandomizedRandomSampleConsensus<PointXYZRGBNormal>(sub_module_RandomizedRandomSampleConsensus, "PointXYZRGBNormal");
}