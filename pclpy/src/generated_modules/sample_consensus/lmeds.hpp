
#include <pcl/sample_consensus/lmeds.h>



template <typename PointT>
void defineSampleConsensusLeastMedianSquares(py::module &m, std::string const & suffix) {
    using Class = pcl::LeastMedianSquares<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::SampleConsensus<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<pcl::SampleConsensusModel<PointT>::Ptr>(), "model"_a);
    cls.def(py::init<pcl::SampleConsensusModel<PointT>::Ptr, double>(), "model"_a, "threshold"_a);
    cls.def("computeModel", &Class::computeModel, "debug_verbosity_level"_a=0);
        
}

void defineSampleConsensusLmedsFunctions(py::module &m) {
}

void defineSampleConsensusLmedsClasses(py::module &sub_module) {
    py::module sub_module_LeastMedianSquares = sub_module.def_submodule("LeastMedianSquares", "Submodule LeastMedianSquares");
    defineSampleConsensusLeastMedianSquares<pcl::PointXYZ>(sub_module_LeastMedianSquares, "PointXYZ");
    defineSampleConsensusLeastMedianSquares<pcl::PointXYZI>(sub_module_LeastMedianSquares, "PointXYZI");
    defineSampleConsensusLeastMedianSquares<pcl::PointXYZRGB>(sub_module_LeastMedianSquares, "PointXYZRGB");
    defineSampleConsensusLeastMedianSquares<pcl::PointXYZRGBA>(sub_module_LeastMedianSquares, "PointXYZRGBA");
    defineSampleConsensusLeastMedianSquares<pcl::PointXYZRGBNormal>(sub_module_LeastMedianSquares, "PointXYZRGBNormal");
}