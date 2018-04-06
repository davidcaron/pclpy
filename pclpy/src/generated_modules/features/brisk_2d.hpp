
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/brisk_2d.h>



template <typename PointInT, 
            typename PointOutT  = pcl::BRISKSignature512, 
            typename KeypointT  = pcl::PointWithScale,
            typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineFeaturesBRISK2DEstimation(py::module &m, std::string const & suffix) {
    using Class = BRISK2DEstimation<PointInT, PointOutT, KeypointT, IntensityT>;
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
    cls.def("set_rotation_invariance", &Class::setRotationInvariance);
    cls.def("set_scale_invariance", &Class::setScaleInvariance);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("set_keypoints", &Class::setKeypoints);
    cls.def("compute", &Class::compute);
        
}

void defineFeaturesBrisk2dClasses(py::module &sub_module) {
}