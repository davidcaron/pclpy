
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/keypoints/brisk_2d.h>



template <typename PointInT, typename PointOutT = pcl::PointWithScale, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineKeypointsBriskKeypoint2D(py::module &m, std::string const & suffix) {
    using Class = BriskKeypoint2D<PointInT, PointOutT, IntensityT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, Keypoint<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<int, int>(), "octaves"_a=4, "threshold"_a=60);
    cls.def_property("threshold", &Class::getThreshold, &Class::setThreshold);
    cls.def_property("octaves", &Class::getOctaves, &Class::setOctaves);
    cls.def_property("remove_invalid3_d_keypoints", &Class::getRemoveInvalid3DKeypoints, &Class::setRemoveInvalid3DKeypoints);
    cls.def("bilinear_interpolation", &Class::bilinearInterpolation);
        
}

void defineKeypointsBrisk2dClasses(py::module &sub_module) {
}