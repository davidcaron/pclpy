
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/recognition/cg/correspondence_grouping.h>



template <typename PointModelT, typename PointSceneT>
void defineRecognitionCorrespondenceGrouping(py::module &m, std::string const & suffix) {
    using Class = pcl::CorrespondenceGrouping<PointModelT, PointSceneT>;
    using SceneCloud = Class::SceneCloud;
    using SceneCloudPtr = Class::SceneCloudPtr;
    using SceneCloudConstPtr = Class::SceneCloudConstPtr;
    py::class_<Class, pcl::PCLBase<pcl::PointModelT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("cluster", &Class::cluster, "clustered_corrs"_a);
    cls.def("setSceneCloud", &Class::setSceneCloud, "scene"_a);
    cls.def("setModelSceneCorrespondences", &Class::setModelSceneCorrespondences, "corrs"_a);
    cls.def("getSceneCloud", &Class::getSceneCloud);
    cls.def("getModelSceneCorrespondences", &Class::getModelSceneCorrespondences);
    cls.def("getCharacteristicScales", &Class::getCharacteristicScales);
        
}

void defineRecognitionCorrespondenceGroupingFunctions(py::module &m) {
}

void defineRecognitionCorrespondenceGroupingClasses(py::module &sub_module) {
}