
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/sample_consensus/sac_model.h>



template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
void defineSampleConsensusFunctor(py::module &m, std::string const & suffix) {
    using Class = pcl::Functor<_Scalar>;
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
    using Class = pcl::SampleConsensusModel<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using SearchPtr = Class::SearchPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("computeModelCoefficients", &Class::computeModelCoefficients, "samples"_a, "model_coefficients"_a);
    cls.def("optimizeModelCoefficients", &Class::optimizeModelCoefficients, "inliers"_a, "model_coefficients"_a, "optimized_coefficients"_a);
    cls.def("selectWithinDistance", &Class::selectWithinDistance, "model_coefficients"_a, "threshold"_a, "inliers"_a);
    cls.def("countWithinDistance", &Class::countWithinDistance, "model_coefficients"_a, "threshold"_a);
    cls.def("projectPoints", &Class::projectPoints, "inliers"_a, "model_coefficients"_a, "projected_points"_a, "copy_data_fields"_a=true);
    cls.def("doSamplesVerifyModel", &Class::doSamplesVerifyModel, "indices"_a, "model_coefficients"_a, "threshold"_a);
    cls.def("computeVariance", py::overload_cast<const std::vector<double> &> (&Class::computeVariance), "error_sqr_dists"_a);
    cls.def("computeVariance", py::overload_cast<> (&Class::computeVariance));
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setIndices", py::overload_cast<const boost::shared_ptr<std::vector<int> > &> (&Class::setIndices), "indices"_a);
    cls.def("setIndices", py::overload_cast<const std::vector<int> &> (&Class::setIndices), "indices"_a);
    cls.def("setRadiusLimits", &Class::setRadiusLimits, "min_radius"_a, "max_radius"_a);
    cls.def("setSamplesMaxDist", &Class::setSamplesMaxDist, "radius"_a, "search"_a);
    cls.def("getSamples", &Class::getSamples, "iterations"_a, "samples"_a);
    cls.def("getDistancesToModel", &Class::getDistancesToModel, "model_coefficients"_a, "distances"_a);
    cls.def("getInputCloud", &Class::getInputCloud);
    cls.def("getIndices", &Class::getIndices);
    cls.def("getModelType", &Class::getModelType);
    cls.def("getClassName", &Class::getClassName);
    cls.def("getSampleSize", &Class::getSampleSize);
    cls.def("getModelSize", &Class::getModelSize);
    cls.def("getRadiusLimits", &Class::getRadiusLimits, "min_radius"_a, "max_radius"_a);
    cls.def("getSamplesMaxDist", &Class::getSamplesMaxDist, "radius"_a);
        
}

template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelFromNormals(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelFromNormals<PointT, PointNT>;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setNormalDistanceWeight", &Class::setNormalDistanceWeight, "w"_a);
    cls.def("setInputNormals", &Class::setInputNormals, "normals"_a);
    cls.def("getNormalDistanceWeight", &Class::getNormalDistanceWeight);
    cls.def("getInputNormals", &Class::getInputNormals);
        
}

void defineSampleConsensusSacModelFunctions(py::module &m) {
}

void defineSampleConsensusSacModelClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModel = sub_module.def_submodule("SampleConsensusModel", "Submodule SampleConsensusModel");
    defineSampleConsensusSampleConsensusModel<pcl::PointXYZ>(sub_module_SampleConsensusModel, "PointXYZ");
    defineSampleConsensusSampleConsensusModel<pcl::PointXYZI>(sub_module_SampleConsensusModel, "PointXYZI");
    defineSampleConsensusSampleConsensusModel<pcl::PointXYZRGB>(sub_module_SampleConsensusModel, "PointXYZRGB");
    defineSampleConsensusSampleConsensusModel<pcl::PointXYZRGBA>(sub_module_SampleConsensusModel, "PointXYZRGBA");
    defineSampleConsensusSampleConsensusModel<pcl::PointXYZRGBNormal>(sub_module_SampleConsensusModel, "PointXYZRGBNormal");
    py::module sub_module_SampleConsensusModelFromNormals = sub_module.def_submodule("SampleConsensusModelFromNormals", "Submodule SampleConsensusModelFromNormals");
    defineSampleConsensusSampleConsensusModelFromNormals<pcl::PointXYZ, pcl::Normal>(sub_module_SampleConsensusModelFromNormals, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelFromNormals<pcl::PointXYZI, pcl::Normal>(sub_module_SampleConsensusModelFromNormals, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelFromNormals<pcl::PointXYZRGB, pcl::Normal>(sub_module_SampleConsensusModelFromNormals, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelFromNormals<pcl::PointXYZRGBA, pcl::Normal>(sub_module_SampleConsensusModelFromNormals, "PointXYZRGBA_Normal");
}