
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model.h>



template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
void defineSampleConsensusFunctor(py::module &m, std::string const & suffix) {
    using Class = Functor<_Scalar>;
    using Scalar = Class::Scalar;
    using ValueType = Class::ValueType;
    using InputType = Class::InputType;
    using JacobianType = Class::JacobianType;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<int>(), "m_data_points"_a);
    cls.def("values", &Class::values);
        
}

template <typename PointT>
void defineSampleConsensusSampleConsensusModel(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModel<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using SearchPtr = Class::SearchPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("input_cloud", &Class::getInputCloud, &Class::setInputCloud);
    cls.def_property("radius_limits", &Class::getRadiusLimits, &Class::setRadiusLimits);
    cls.def_property("samples_max_dist", &Class::getSamplesMaxDist, &Class::setSamplesMaxDist);
    cls.def("compute_variance", py::overload_cast<const std::vector<double> &> (&Class::computeVariance));
    cls.def("compute_variance", py::overload_cast<> (&Class::computeVariance));
    cls.def("compute_model_coefficients", &Class::computeModelCoefficients);
    cls.def("optimize_model_coefficients", &Class::optimizeModelCoefficients);
    cls.def("select_within_distance", &Class::selectWithinDistance);
    cls.def("count_within_distance", &Class::countWithinDistance);
    cls.def("project_points", &Class::projectPoints);
    cls.def("do_samples_verify_model", &Class::doSamplesVerifyModel);
    cls.def("set_indices", py::overload_cast<const boost::shared_ptr<std::vector<int> > &> (&Class::setIndices));
    cls.def("set_indices", py::overload_cast<const std::vector<int> &> (&Class::setIndices));
        
}

template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelFromNormals(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelFromNormals<PointT, PointNT>;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("normal_distance_weight", &Class::getNormalDistanceWeight, &Class::setNormalDistanceWeight);
    cls.def_property("input_normals", &Class::getInputNormals, &Class::setInputNormals);
        
}

void defineSampleConsensusSacModelClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModel = sub_module.def_submodule("SampleConsensusModel", "Submodule SampleConsensusModel");
    defineSampleConsensusSampleConsensusModel<PointXYZ>(sub_module_SampleConsensusModel, "PointXYZ");
    defineSampleConsensusSampleConsensusModel<PointXYZI>(sub_module_SampleConsensusModel, "PointXYZI");
    defineSampleConsensusSampleConsensusModel<PointXYZRGB>(sub_module_SampleConsensusModel, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModel<PointXYZRGBA>(sub_module_SampleConsensusModel, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModel<PointXYZRGBNormal>(sub_module_SampleConsensusModel, "PointXYZRGBNormal");
    py::module sub_module_SampleConsensusModelFromNormals = sub_module.def_submodule("SampleConsensusModelFromNormals", "Submodule SampleConsensusModelFromNormals");
    defineSampleConsensusSampleConsensusModelFromNormals<PointXYZ, Normal>(sub_module_SampleConsensusModelFromNormals, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelFromNormals<PointXYZI, Normal>(sub_module_SampleConsensusModelFromNormals, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelFromNormals<PointXYZRGB, Normal>(sub_module_SampleConsensusModelFromNormals, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelFromNormals<PointXYZRGBA, Normal>(sub_module_SampleConsensusModelFromNormals, "PointXYZRGBA_Normal");
}