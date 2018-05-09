
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/sample_consensus/ransac.h>



template <typename PointT>
void defineSampleConsensusRandomSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = pcl::RandomSampleConsensus<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<pcl::SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<pcl::SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def("computeModel", &Class::computeModel, "debug_verbosity_level"_a=0);
        
}

void defineSampleConsensusRansacFunctions(py::module &m) {
}

void defineSampleConsensusRansacClasses(py::module &sub_module) {
    py::module sub_module_RandomSampleConsensus = sub_module.def_submodule("RandomSampleConsensus", "Submodule RandomSampleConsensus");
    defineSampleConsensusRandomSampleConsensus<pcl::PointNormal>(sub_module_RandomSampleConsensus, "PointNormal");
    defineSampleConsensusRandomSampleConsensus<pcl::PointXYZ>(sub_module_RandomSampleConsensus, "PointXYZ");
    defineSampleConsensusRandomSampleConsensus<pcl::PointXYZI>(sub_module_RandomSampleConsensus, "PointXYZI");
    defineSampleConsensusRandomSampleConsensus<pcl::PointXYZRGB>(sub_module_RandomSampleConsensus, "PointXYZRGB");
    defineSampleConsensusRandomSampleConsensus<pcl::PointXYZRGBA>(sub_module_RandomSampleConsensus, "PointXYZRGBA");
    defineSampleConsensusRandomSampleConsensus<pcl::PointXYZRGBNormal>(sub_module_RandomSampleConsensus, "PointXYZRGBNormal");
}