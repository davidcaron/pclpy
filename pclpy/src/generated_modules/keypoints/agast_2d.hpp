
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/keypoints/agast_2d.h>



template <typename PointInT, typename PointOutT, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineKeypointsAgastKeypoint2DBase(py::module &m, std::string const & suffix) {
    using Class = pcl::AgastKeypoint2DBase<PointInT, PointOutT, IntensityT>;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using AgastDetectorPtr = Class::AgastDetectorPtr;
    py::class_<Class, pcl::Keypoint<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("setThreshold", &Class::setThreshold, "threshold"_a);
    cls.def("setMaxKeypoints", &Class::setMaxKeypoints, "nr_max_keypoints"_a);
    cls.def("setMaxDataValue", &Class::setMaxDataValue, "bmax"_a);
    cls.def("setNonMaxSuppression", &Class::setNonMaxSuppression, "enabled"_a);
    cls.def("setAgastDetector", &Class::setAgastDetector, "detector"_a);
    cls.def("getThreshold", &Class::getThreshold);
    cls.def("getMaxKeypoints", &Class::getMaxKeypoints);
    cls.def("getMaxDataValue", &Class::getMaxDataValue);
    cls.def("getNonMaxSuppression", &Class::getNonMaxSuppression);
    cls.def("getAgastDetector", &Class::getAgastDetector);
        
}

template <typename PointInT, typename PointOutT = pcl::PointUV>
void defineKeypointsAgastKeypoint2D(py::module &m, std::string const & suffix) {
    using Class = pcl::AgastKeypoint2D<PointInT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::AgastKeypoint2DBase<PointInT, PointOutT, pcl::common::IntensityFieldAccessor<PointInT>>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineKeypointsAgast2dFunctions(py::module &m) {
}

void defineKeypointsAgast2dClasses(py::module &sub_module) {
}