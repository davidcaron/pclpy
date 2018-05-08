
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

#include <pcl/keypoints/susan.h>



template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal, typename IntensityT= pcl::common::IntensityFieldAccessor<PointInT> >
void defineKeypointsSUSANKeypoint(py::module &m, std::string const & suffix) {
    using Class = pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT=>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    py::class_<Class, pcl::Keypoint<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<float, float, float, float>(), "radius"_a=0.01f, "distance_threshold"_a=0.001f, "angular_threshold"_a=0.0001f, "intensity_threshold"_a=7.0f);
    cls.def("setRadius", &Class::setRadius, "radius"_a);
    cls.def("setDistanceThreshold", &Class::setDistanceThreshold, "distance_threshold"_a);
    cls.def("setAngularThreshold", &Class::setAngularThreshold, "angular_threshold"_a);
    cls.def("setIntensityThreshold", &Class::setIntensityThreshold, "intensity_threshold"_a);
    cls.def("setNormals", &Class::setNormals, "normals"_a);
    cls.def("setSearchSurface", &Class::setSearchSurface, "cloud"_a);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a);
    cls.def("setNonMaxSupression", &Class::setNonMaxSupression, "nonmax"_a);
    cls.def("setGeometricValidation", &Class::setGeometricValidation, "validate"_a);
        
}

void defineKeypointsSusanFunctions(py::module &m) {
}

void defineKeypointsSusanClasses(py::module &sub_module) {
}