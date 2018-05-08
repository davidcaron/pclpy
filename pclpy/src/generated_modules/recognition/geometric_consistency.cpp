
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/recognition/cg/geometric_consistency.h>



template<typename PointModelT, typename PointSceneT>
void defineRecognitionGeometricConsistencyGrouping(py::module &m, std::string const & suffix) {
    using Class = pcl::GeometricConsistencyGrouping<PointModelT, PointSceneT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using SceneCloudConstPtr = Class::SceneCloudConstPtr;
    py::class_<Class, pcl::CorrespondenceGrouping<pcl::PointModelT, pcl::PointSceneT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("recognize", py::overload_cast<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &> (&Class::recognize), "transformations"_a);
    cls.def("recognize", py::overload_cast<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &, std::vector<pcl::Correspondences> &> (&Class::recognize), "transformations"_a, "clustered_corrs"_a);
    cls.def("setGCThreshold", &Class::setGCThreshold, "threshold"_a);
    cls.def("setGCSize", &Class::setGCSize, "gc_size"_a);
    cls.def("getGCThreshold", &Class::getGCThreshold);
    cls.def("getGCSize", &Class::getGCSize);
        
}

void defineRecognitionGeometricConsistencyFunctions(py::module &m) {
}

void defineRecognitionGeometricConsistencyClasses(py::module &sub_module) {
}