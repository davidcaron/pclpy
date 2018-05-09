
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/segmentation/sac_segmentation.h>



template <typename PointT>
void defineSegmentationSACSegmentation(py::module &m, std::string const & suffix) {
    using Class = pcl::SACSegmentation<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using SearchPtr = Class::SearchPtr;
    using SampleConsensusPtr = Class::SampleConsensusPtr;
    using SampleConsensusModelPtr = Class::SampleConsensusModelPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "random"_a=false);
    cls.def("segment", py::overload_cast<pcl::PointIndices &, pcl::ModelCoefficients &> (&Class::segment), "inliers"_a, "model_coefficients"_a);
    cls.def("setModelType", &Class::setModelType, "model"_a);
    cls.def("setMethodType", &Class::setMethodType, "method"_a);
    cls.def("setDistanceThreshold", &Class::setDistanceThreshold, "threshold"_a);
    cls.def("setMaxIterations", &Class::setMaxIterations, "max_iterations"_a);
    cls.def("setProbability", &Class::setProbability, "probability"_a);
    cls.def("setOptimizeCoefficients", &Class::setOptimizeCoefficients, "optimize"_a);
    cls.def("setRadiusLimits", &Class::setRadiusLimits, "min_radius"_a, "max_radius"_a);
    cls.def("setSamplesMaxDist", &Class::setSamplesMaxDist, "radius"_a, "search"_a);
    cls.def("setAxis", &Class::setAxis, "ax"_a);
    cls.def("setEpsAngle", &Class::setEpsAngle, "ea"_a);
    cls.def("getModelType", &Class::getModelType);
    cls.def("getMethod", &Class::getMethod);
    cls.def("getModel", &Class::getModel);
    cls.def("getMethodType", &Class::getMethodType);
    cls.def("getDistanceThreshold", &Class::getDistanceThreshold);
    cls.def("getMaxIterations", &Class::getMaxIterations);
    cls.def("getProbability", &Class::getProbability);
    cls.def("getOptimizeCoefficients", &Class::getOptimizeCoefficients);
    cls.def("getRadiusLimits", &Class::getRadiusLimits, "min_radius"_a, "max_radius"_a);
    cls.def("getSamplesMaxDist", &Class::getSamplesMaxDist, "radius"_a);
    cls.def("getAxis", &Class::getAxis);
    cls.def("getEpsAngle", &Class::getEpsAngle);
        
}

template <typename PointT, typename PointNT>
void defineSegmentationSACSegmentationFromNormals(py::module &m, std::string const & suffix) {
    using Class = pcl::SACSegmentationFromNormals<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using SampleConsensusPtr = Class::SampleConsensusPtr;
    using SampleConsensusModelPtr = Class::SampleConsensusModelPtr;
    using SampleConsensusModelFromNormalsPtr = Class::SampleConsensusModelFromNormalsPtr;
    py::class_<Class, pcl::SACSegmentation<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "random"_a=false);
    cls.def("setInputNormals", &Class::setInputNormals, "normals"_a);
    cls.def("setNormalDistanceWeight", &Class::setNormalDistanceWeight, "distance_weight"_a);
    cls.def("setMinMaxOpeningAngle", &Class::setMinMaxOpeningAngle, "min_angle"_a, "max_angle"_a);
    cls.def("setDistanceFromOrigin", &Class::setDistanceFromOrigin, "d"_a);
    cls.def("getInputNormals", &Class::getInputNormals);
    cls.def("getNormalDistanceWeight", &Class::getNormalDistanceWeight);
    cls.def("getMinMaxOpeningAngle", &Class::getMinMaxOpeningAngle, "min_angle"_a, "max_angle"_a);
    cls.def("getDistanceFromOrigin", &Class::getDistanceFromOrigin);
        
}

void defineSegmentationSacSegmentationFunctions(py::module &m) {
}

void defineSegmentationSacSegmentationClasses(py::module &sub_module) {
    py::module sub_module_SACSegmentation = sub_module.def_submodule("SACSegmentation", "Submodule SACSegmentation");
    defineSegmentationSACSegmentation<pcl::PointXYZ>(sub_module_SACSegmentation, "PointXYZ");
    defineSegmentationSACSegmentation<pcl::PointXYZI>(sub_module_SACSegmentation, "PointXYZI");
    defineSegmentationSACSegmentation<pcl::PointXYZRGB>(sub_module_SACSegmentation, "PointXYZRGB");
    defineSegmentationSACSegmentation<pcl::PointXYZRGBA>(sub_module_SACSegmentation, "PointXYZRGBA");
    defineSegmentationSACSegmentation<pcl::PointXYZRGBNormal>(sub_module_SACSegmentation, "PointXYZRGBNormal");
    py::module sub_module_SACSegmentationFromNormals = sub_module.def_submodule("SACSegmentationFromNormals", "Submodule SACSegmentationFromNormals");
    defineSegmentationSACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>(sub_module_SACSegmentationFromNormals, "PointXYZ_Normal");
    defineSegmentationSACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal>(sub_module_SACSegmentationFromNormals, "PointXYZI_Normal");
    defineSegmentationSACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal>(sub_module_SACSegmentationFromNormals, "PointXYZRGB_Normal");
    defineSegmentationSACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal>(sub_module_SACSegmentationFromNormals, "PointXYZRGBA_Normal");
}