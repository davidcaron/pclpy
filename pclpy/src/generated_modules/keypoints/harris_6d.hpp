
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/keypoints/harris_6d.h>



template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
void defineKeypointsHarrisKeypoint6D(py::module &m, std::string const & suffix) {
    using Class = HarrisKeypoint6D<PointInT, PointOutT, NormalT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, Keypoint<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<float, float>(), "radius"_a=0.01, "threshold"_a=0.0);
    cls.def("set_radius", &Class::setRadius);
    cls.def("set_threshold", &Class::setThreshold);
    cls.def("set_non_max_supression", &Class::setNonMaxSupression);
    cls.def("set_refine", &Class::setRefine);
    cls.def("set_search_surface", &Class::setSearchSurface);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineKeypointsHarris6dClasses(py::module &sub_module) {
}