
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/prosac.h>



template<typename PointT>
void defineSampleConsensusProgressiveSampleConsensus(py::module &m, std::string const & suffix) {
    using Class = ProgressiveSampleConsensus<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def("compute_model", &Class::computeModel);
        
}

void defineSampleConsensusProsacClasses(py::module &sub_module) {
    py::module sub_module_ProgressiveSampleConsensus = sub_module.def_submodule("ProgressiveSampleConsensus", "Submodule ProgressiveSampleConsensus");
    defineSampleConsensusProgressiveSampleConsensus<PointXYZ>(sub_module_ProgressiveSampleConsensus, "PointXYZ");
    defineSampleConsensusProgressiveSampleConsensus<PointXYZI>(sub_module_ProgressiveSampleConsensus, "PointXYZI");
    defineSampleConsensusProgressiveSampleConsensus<PointXYZRGB>(sub_module_ProgressiveSampleConsensus, "PointXYZRGB");
    defineSampleConsensusProgressiveSampleConsensus<PointXYZRGBA>(sub_module_ProgressiveSampleConsensus, "PointXYZRGBA");
    defineSampleConsensusProgressiveSampleConsensus<PointXYZRGBNormal>(sub_module_ProgressiveSampleConsensus, "PointXYZRGBNormal");
}