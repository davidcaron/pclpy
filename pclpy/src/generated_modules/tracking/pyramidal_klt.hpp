
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/pyramidal_klt.h>

using namespace pcl::tracking;


template<typename PointInT, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineTrackingPyramidalKLTTracker(py::module &m, std::string const & suffix) {
    using Class = tracking::PyramidalKLTTracker<PointInT, IntensityT>;
    using TrackerBase = Class::TrackerBase;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using FloatImage = Class::FloatImage;
    using FloatImagePtr = Class::FloatImagePtr;
    using FloatImageConstPtr = Class::FloatImageConstPtr;
    py::class_<Class, Tracker<PointInT,Eigen::Affine3f>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<int, int, int>(), "nb_levels"_a=5, "tracking_window_width"_a=7, "tracking_window_height"_a=7);
    cls.def_property("number_of_pyramid_levels", &Class::getNumberOfPyramidLevels, &Class::setNumberOfPyramidLevels);
    cls.def_property("accuracy", &Class::getAccuracy, &Class::setAccuracy);
    cls.def_property("epsilon", &Class::getEpsilon, &Class::setEpsilon);
    cls.def_property("number_of_keypoints", &Class::getNumberOfKeypoints, &Class::setNumberOfKeypoints);
    cls.def("set_tracking_window_size", &Class::setTrackingWindowSize);
    cls.def_property("tracking_window_width", &Class::getTrackingWindowWidth, &Class::setTrackingWindowWidth);
    cls.def_property("tracking_window_height", &Class::getTrackingWindowHeight, &Class::setTrackingWindowHeight);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
    cls.def_property("max_iterations_number", &Class::getMaxIterationsNumber, &Class::setMaxIterationsNumber);
    cls.def("set_points_to_track", py::overload_cast<const pcl::PointIndicesConstPtr &> (&Class::setPointsToTrack));
    cls.def("set_points_to_track", py::overload_cast<const pcl::PointCloud<pcl::PointUV>::ConstPtr &> (&Class::setPointsToTrack));
        
}

void defineTrackingPyramidalKltClasses(py::module &sub_module) {
}