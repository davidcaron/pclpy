
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/sample_consensus/prosac.h>



template<typename PointT>
void defineSampleConsensusProgressiveSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = pcl::ProgressiveSampleConsensus<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<pcl::SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<pcl::SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def("computeModel", &Class::computeModel, "debug_verbosity_level"_a=0);
        
}

void defineSampleConsensusProsacFunctions(py::module &m) {
}

void defineSampleConsensusProsacClasses(py::module &sub_module) {
    py::module sub_module_ProgressiveSampleConsensus = sub_module.def_submodule("ProgressiveSampleConsensus", "Submodule ProgressiveSampleConsensus");
    defineSampleConsensusProgressiveSampleConsensus<pcl::PointXYZ>(sub_module_ProgressiveSampleConsensus, "PointXYZ");
    defineSampleConsensusProgressiveSampleConsensus<pcl::PointXYZI>(sub_module_ProgressiveSampleConsensus, "PointXYZI");
    defineSampleConsensusProgressiveSampleConsensus<pcl::PointXYZRGB>(sub_module_ProgressiveSampleConsensus, "PointXYZRGB");
    defineSampleConsensusProgressiveSampleConsensus<pcl::PointXYZRGBA>(sub_module_ProgressiveSampleConsensus, "PointXYZRGBA");
    defineSampleConsensusProgressiveSampleConsensus<pcl::PointXYZRGBNormal>(sub_module_ProgressiveSampleConsensus, "PointXYZRGBNormal");
}