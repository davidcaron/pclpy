
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/recognition/cg/hough_3d.h>

using namespace pcl::recognition;


template<typename PointModelT, typename PointSceneT, typename PointModelRfT = pcl::ReferenceFrame, typename PointSceneRfT = pcl::ReferenceFrame>
void defineRecognitionHough3DGrouping(py::module &m, std::string const & suffix) {
    using Class = pcl::Hough3DGrouping<PointModelT, PointSceneT, PointModelRfT, PointSceneRfT>;
    using ModelRfCloud = Class::ModelRfCloud;
    using ModelRfCloudPtr = Class::ModelRfCloudPtr;
    using ModelRfCloudConstPtr = Class::ModelRfCloudConstPtr;
    using SceneRfCloud = Class::SceneRfCloud;
    using SceneRfCloudPtr = Class::SceneRfCloudPtr;
    using SceneRfCloudConstPtr = Class::SceneRfCloudConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using SceneCloudConstPtr = Class::SceneCloudConstPtr;
    py::class_<Class, pcl::CorrespondenceGrouping<pcl::PointModelT, pcl::PointSceneT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("train", &Class::train);
    cls.def("recognize", py::overload_cast<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &> (&Class::recognize), "transformations"_a);
    cls.def("recognize", py::overload_cast<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &, std::vector<pcl::Correspondences> &> (&Class::recognize), "transformations"_a, "clustered_corrs"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setInputRf", &Class::setInputRf, "input_rf"_a);
    cls.def("setSceneCloud", &Class::setSceneCloud, "scene"_a);
    cls.def("setSceneRf", &Class::setSceneRf, "scene_rf"_a);
    cls.def("setModelSceneCorrespondences", &Class::setModelSceneCorrespondences, "corrs"_a);
    cls.def("setHoughThreshold", &Class::setHoughThreshold, "threshold"_a);
    cls.def("setHoughBinSize", &Class::setHoughBinSize, "bin_size"_a);
    cls.def("setUseInterpolation", &Class::setUseInterpolation, "use_interpolation"_a);
    cls.def("setUseDistanceWeight", &Class::setUseDistanceWeight, "use_distance_weight"_a);
    cls.def("setLocalRfNormalsSearchRadius", &Class::setLocalRfNormalsSearchRadius, "local_rf_normals_search_radius"_a);
    cls.def("setLocalRfSearchRadius", &Class::setLocalRfSearchRadius, "local_rf_search_radius"_a);
    cls.def("getInputRf", &Class::getInputRf);
    cls.def("getSceneRf", &Class::getSceneRf);
    cls.def("getHoughThreshold", &Class::getHoughThreshold);
    cls.def("getHoughBinSize", &Class::getHoughBinSize);
    cls.def("getUseInterpolation", &Class::getUseInterpolation);
    cls.def("getUseDistanceWeight", &Class::getUseDistanceWeight);
    cls.def("getLocalRfNormalsSearchRadius", &Class::getLocalRfNormalsSearchRadius);
    cls.def("getLocalRfSearchRadius", &Class::getLocalRfSearchRadius);
        
}

void defineRecognitionHoughSpace3D(py::module &m) {
    using Class = pcl::recognition::HoughSpace3D;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "HoughSpace3D");
    cls.def(py::init<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>(), "min_coord"_a, "bin_size"_a, "max_coord"_a);
    cls.def("reset", &Class::reset);
    cls.def("vote", &Class::vote, "single_vote_coord"_a, "weight"_a, "voter_id"_a);
    cls.def("voteInt", &Class::voteInt, "single_vote_coord"_a, "weight"_a, "voter_id"_a);
    cls.def("findMaxima", &Class::findMaxima, "min_threshold"_a, "maxima_values"_a, "maxima_voter_ids"_a);
}

void defineRecognitionHough3dFunctions(py::module &m) {
}

void defineRecognitionHough3dClasses(py::module &sub_module) {
    defineRecognitionHoughSpace3D(sub_module);
}