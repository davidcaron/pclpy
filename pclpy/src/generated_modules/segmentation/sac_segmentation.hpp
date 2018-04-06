
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/sac_segmentation.h>



template <typename PointT>
void defineSegmentationSACSegmentation(py::module &m, std::string const & suffix) {
    using Class = SACSegmentation<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using SearchPtr = Class::SearchPtr;
    using SampleConsensusPtr = Class::SampleConsensusPtr;
    using SampleConsensusModelPtr = Class::SampleConsensusModelPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "random"_a=false);
    cls.def_property("model_type", &Class::getModelType, &Class::setModelType);
    cls.def_property("method_type", &Class::getMethodType, &Class::setMethodType);
    cls.def_property("distance_threshold", &Class::getDistanceThreshold, &Class::setDistanceThreshold);
    cls.def_property("max_iterations", &Class::getMaxIterations, &Class::setMaxIterations);
    cls.def_property("probability", &Class::getProbability, &Class::setProbability);
    cls.def_property("optimize_coefficients", &Class::getOptimizeCoefficients, &Class::setOptimizeCoefficients);
    cls.def_property("radius_limits", &Class::getRadiusLimits, &Class::setRadiusLimits);
    cls.def_property("samples_max_dist", &Class::getSamplesMaxDist, &Class::setSamplesMaxDist);
    cls.def_property("axis", &Class::getAxis, &Class::setAxis);
    cls.def_property("eps_angle", &Class::getEpsAngle, &Class::setEpsAngle);
    cls.def("segment", py::overload_cast<PointIndices &, ModelCoefficients &> (&Class::segment));
        
}

template <typename PointT, typename PointNT>
void defineSegmentationSACSegmentationFromNormals(py::module &m, std::string const & suffix) {
    using Class = SACSegmentationFromNormals<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using SampleConsensusPtr = Class::SampleConsensusPtr;
    using SampleConsensusModelPtr = Class::SampleConsensusModelPtr;
    using SampleConsensusModelFromNormalsPtr = Class::SampleConsensusModelFromNormalsPtr;
    py::class_<Class, SACSegmentation<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "random"_a=false);
    cls.def_property("input_normals", &Class::getInputNormals, &Class::setInputNormals);
    cls.def_property("normal_distance_weight", &Class::getNormalDistanceWeight, &Class::setNormalDistanceWeight);
    cls.def_property("min_max_opening_angle", &Class::getMinMaxOpeningAngle, &Class::setMinMaxOpeningAngle);
    cls.def_property("distance_from_origin", &Class::getDistanceFromOrigin, &Class::setDistanceFromOrigin);
        
}

void defineSegmentationSacSegmentationClasses(py::module &sub_module) {
    py::module sub_module_SACSegmentation = sub_module.def_submodule("SACSegmentation", "Submodule SACSegmentation");
    defineSegmentationSACSegmentation<PointXYZ>(sub_module_SACSegmentation, "PointXYZ");
    defineSegmentationSACSegmentation<PointXYZI>(sub_module_SACSegmentation, "PointXYZI");
    defineSegmentationSACSegmentation<PointXYZRGB>(sub_module_SACSegmentation, "PointXYZRGB");
    defineSegmentationSACSegmentation<PointXYZRGBA>(sub_module_SACSegmentation, "PointXYZRGBA");
    defineSegmentationSACSegmentation<PointXYZRGBNormal>(sub_module_SACSegmentation, "PointXYZRGBNormal");
    py::module sub_module_SACSegmentationFromNormals = sub_module.def_submodule("SACSegmentationFromNormals", "Submodule SACSegmentationFromNormals");
    defineSegmentationSACSegmentationFromNormals<PointXYZ, Normal>(sub_module_SACSegmentationFromNormals, "PointXYZ_Normal");
    defineSegmentationSACSegmentationFromNormals<PointXYZI, Normal>(sub_module_SACSegmentationFromNormals, "PointXYZI_Normal");
    defineSegmentationSACSegmentationFromNormals<PointXYZRGB, Normal>(sub_module_SACSegmentationFromNormals, "PointXYZRGB_Normal");
    defineSegmentationSACSegmentationFromNormals<PointXYZRGBA, Normal>(sub_module_SACSegmentationFromNormals, "PointXYZRGBA_Normal");
}