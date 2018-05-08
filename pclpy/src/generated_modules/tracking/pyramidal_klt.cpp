
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

#include <pcl/tracking/pyramidal_klt.h>

using namespace pcl::tracking;


template<typename PointInT, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineTrackingPyramidalKLTTracker(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::PyramidalKLTTracker<PointInT, IntensityT>;
    using TrackerBase = Class::TrackerBase;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using FloatImage = Class::FloatImage;
    using FloatImagePtr = Class::FloatImagePtr;
    using FloatImageConstPtr = Class::FloatImageConstPtr;
    py::class_<Class, pcl::tracking::Tracker<PointInT, Eigen::Affine3f>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<int, int, int>(), "nb_levels"_a=5, "tracking_window_width"_a=7, "tracking_window_height"_a=7);
    cls.def("setNumberOfPyramidLevels", &Class::setNumberOfPyramidLevels, "levels"_a);
    cls.def("setAccuracy", &Class::setAccuracy, "accuracy"_a);
    cls.def("setEpsilon", &Class::setEpsilon, "epsilon"_a);
    cls.def("setNumberOfKeypoints", &Class::setNumberOfKeypoints, "number"_a);
    cls.def("setTrackingWindowSize", &Class::setTrackingWindowSize, "width"_a, "height"_a);
    cls.def("setTrackingWindowWidth", &Class::setTrackingWindowWidth, "width"_a);
    cls.def("setTrackingWindowHeight", &Class::setTrackingWindowHeight, "height"_a);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
    cls.def("setMaxIterationsNumber", &Class::setMaxIterationsNumber, "max"_a);
    cls.def("setPointsToTrack", py::overload_cast<const pcl::Pointpcl::IndicesConstPtr &> (&Class::setPointsToTrack), "points"_a);
    cls.def("setPointsToTrack", py::overload_cast<const pcl::PointCloud<pcl::PointUV>::ConstPtr &> (&Class::setPointsToTrack), "points"_a);
    cls.def("getNumberOfPyramidLevels", &Class::getNumberOfPyramidLevels);
    cls.def("getAccuracy", &Class::getAccuracy);
    cls.def("getEpsilon", &Class::getEpsilon);
    cls.def("getNumberOfKeypoints", &Class::getNumberOfKeypoints);
    cls.def("getTrackingWindowWidth", &Class::getTrackingWindowWidth);
    cls.def("getTrackingWindowHeight", &Class::getTrackingWindowHeight);
    cls.def("getReferenceCloud", &Class::getReferenceCloud);
    cls.def("getMaxIterationsNumber", &Class::getMaxIterationsNumber);
    cls.def("getTrackedPoints", &Class::getTrackedPoints);
    cls.def("getPointsToTrackStatus", &Class::getPointsToTrackStatus);
    cls.def("getResult", &Class::getResult);
    cls.def("getInitialized", &Class::getInitialized);
        
}

void defineTrackingPyramidalKltFunctions(py::module &m) {
}

void defineTrackingPyramidalKltClasses(py::module &sub_module) {
}