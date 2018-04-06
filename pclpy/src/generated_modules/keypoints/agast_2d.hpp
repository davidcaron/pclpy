
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/keypoints/agast_2d.h>



template <typename PointInT, typename PointOutT, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineKeypointsAgastKeypoint2DBase(py::module &m, std::string const & suffix) {
    using Class = AgastKeypoint2DBase<PointInT, PointOutT, IntensityT>;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using AgastDetectorPtr = Class::AgastDetectorPtr;
    py::class_<Class, Keypoint<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("threshold", &Class::getThreshold, &Class::setThreshold);
    cls.def_property("max_keypoints", &Class::getMaxKeypoints, &Class::setMaxKeypoints);
    cls.def_property("max_data_value", &Class::getMaxDataValue, &Class::setMaxDataValue);
    cls.def_property("non_max_suppression", &Class::getNonMaxSuppression, &Class::setNonMaxSuppression);
    cls.def_property("agast_detector", &Class::getAgastDetector, &Class::setAgastDetector);
        
}

template <typename PointInT, typename PointOutT = pcl::PointUV>
void defineKeypointsAgastKeypoint2D(py::module &m, std::string const & suffix) {
    using Class = AgastKeypoint2D<PointInT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, AgastKeypoint2DBase<PointInT,PointOutT,common::IntensityFieldAccessor<PointInT>>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineKeypointsAgast2dClasses(py::module &sub_module) {
}