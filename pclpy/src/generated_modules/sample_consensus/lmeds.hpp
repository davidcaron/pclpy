
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/lmeds.h>



template <typename PointT>
void defineSampleConsensusLeastMedianSquares(py::module &m, std::string const & suffix) {
    using Class = LeastMedianSquares<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def("compute_model", &Class::computeModel);
        
}

void defineSampleConsensusLmedsClasses(py::module &sub_module) {
    py::module sub_module_LeastMedianSquares = sub_module.def_submodule("LeastMedianSquares", "Submodule LeastMedianSquares");
    defineSampleConsensusLeastMedianSquares<PointXYZ>(sub_module_LeastMedianSquares, "PointXYZ");
    defineSampleConsensusLeastMedianSquares<PointXYZI>(sub_module_LeastMedianSquares, "PointXYZI");
    defineSampleConsensusLeastMedianSquares<PointXYZRGB>(sub_module_LeastMedianSquares, "PointXYZRGB");
    defineSampleConsensusLeastMedianSquares<PointXYZRGBA>(sub_module_LeastMedianSquares, "PointXYZRGBA");
    defineSampleConsensusLeastMedianSquares<PointXYZRGBNormal>(sub_module_LeastMedianSquares, "PointXYZRGBNormal");
}