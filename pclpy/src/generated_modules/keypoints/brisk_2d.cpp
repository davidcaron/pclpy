
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

#include <pcl/keypoints/brisk_2d.h>



template <typename PointInT, typename PointOutT = pcl::PointWithScale, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineKeypointsBriskKeypoint2D(py::module &m, std::string const & suffix) {
    using Class = pcl::BriskKeypoint2D<PointInT, PointOutT, IntensityT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, pcl::Keypoint<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<int, int>(), "octaves"_a=4, "threshold"_a=60);
    cls.def("bilinearInterpolation", &Class::bilinearInterpolation, "cloud"_a, "x"_a, "y"_a, "pt"_a);
    cls.def("setThreshold", &Class::setThreshold, "threshold"_a);
    cls.def("setOctaves", &Class::setOctaves, "octaves"_a);
    cls.def("setRemoveInvalid3DKeypoints", &Class::setRemoveInvalid3DKeypoints, "remove"_a);
    cls.def("getThreshold", &Class::getThreshold);
    cls.def("getOctaves", &Class::getOctaves);
    cls.def("getRemoveInvalid3DKeypoints", &Class::getRemoveInvalid3DKeypoints);
        
}

void defineKeypointsBrisk2dFunctions(py::module &m) {
}

void defineKeypointsBrisk2dClasses(py::module &sub_module) {
}