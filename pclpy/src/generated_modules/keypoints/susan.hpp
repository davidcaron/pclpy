
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/keypoints/susan.h>



template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal, typename IntensityT= pcl::common::IntensityFieldAccessor<PointInT> >
void defineKeypointsSUSANKeypoint(py::module &m, std::string const & suffix) {
    using Class = SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT=>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    py::class_<Class, Keypoint<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<float, float, float, float>(), "radius"_a=0.01f, "distance_threshold"_a=0.001f, "angular_threshold"_a=0.0001f, "intensity_threshold"_a=7.0f);
    cls.def("set_radius", &Class::setRadius);
    cls.def("set_distance_threshold", &Class::setDistanceThreshold);
    cls.def("set_angular_threshold", &Class::setAngularThreshold);
    cls.def("set_intensity_threshold", &Class::setIntensityThreshold);
    cls.def("set_normals", &Class::setNormals);
    cls.def("set_search_surface", &Class::setSearchSurface);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
    cls.def("set_non_max_supression", &Class::setNonMaxSupression);
    cls.def("set_geometric_validation", &Class::setGeometricValidation);
        
}

void defineKeypointsSusanClasses(py::module &sub_module) {
}