
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

#include <pcl/features/brisk_2d.h>



template <typename PointInT, 
            typename PointOutT  = pcl::BRISKSignature512, 
            typename KeypointT  = pcl::PointWithScale,
            typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineFeaturesBRISK2DEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::BRISK2DEstimation<PointInT, PointOutT, KeypointT, IntensityT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudInT = Class::PointCloudInT;
    using PointCloudInTConstPtr = Class::PointCloudInTConstPtr;
    using KeypointPointCloudT = Class::KeypointPointCloudT;
    using KeypointPointCloudTPtr = Class::KeypointPointCloudTPtr;
    using KeypointPointCloudTConstPtr = Class::KeypointPointCloudTConstPtr;
    using PointCloudOutT = Class::PointCloudOutT;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute", &Class::compute, "output"_a);
    cls.def("setRotationInvariance", &Class::setRotationInvariance, "enable"_a);
    cls.def("setScaleInvariance", &Class::setScaleInvariance, "enable"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setKeypoints", &Class::setKeypoints, "keypoints"_a);
        
}

void defineFeaturesBrisk2dFunctions(py::module &m) {
}

void defineFeaturesBrisk2dClasses(py::module &sub_module) {
}