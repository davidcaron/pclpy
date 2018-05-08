
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

#include <pcl/keypoints/harris_6d.h>



template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
void defineKeypointsHarrisKeypoint6D(py::module &m, std::string const & suffix) {
    using Class = pcl::HarrisKeypoint6D<PointInT, PointOutT, NormalT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, pcl::Keypoint<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<float, float>(), "radius"_a=0.01, "threshold"_a=0.0);
    cls.def("setRadius", &Class::setRadius, "radius"_a);
    cls.def("setThreshold", &Class::setThreshold, "threshold"_a);
    cls.def("setNonMaxSupression", &Class::setNonMaxSupression, "bool"_a=false);
    cls.def("setRefine", &Class::setRefine, "do_refine"_a);
    cls.def("setSearchSurface", &Class::setSearchSurface, "cloud"_a);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineKeypointsHarris6dFunctions(py::module &m) {
}

void defineKeypointsHarris6dClasses(py::module &sub_module) {
}