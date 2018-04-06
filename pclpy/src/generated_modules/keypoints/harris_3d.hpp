
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/keypoints/harris_3d.h>



template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
void defineKeypointsHarrisKeypoint3D(py::module &m, std::string const & suffix) {
    using Class = HarrisKeypoint3D<PointInT, PointOutT, NormalT>;
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
    py::enum_<Class::ResponseMethod>(cls, "response_method")
        .value("HARRIS", Class::ResponseMethod::HARRIS)
        .value("NOBLE", Class::ResponseMethod::NOBLE)
        .value("LOWE", Class::ResponseMethod::LOWE)
        .value("TOMASI", Class::ResponseMethod::TOMASI)
        .value("CURVATURE", Class::ResponseMethod::CURVATURE)
        .export_values();
    cls.def(py::init<Class::ResponseMethod, float, float>(), "method"_a=Class::HARRIS, "radius"_a=0.01f, "threshold"_a=0.0f);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("set_method", &Class::setMethod);
    cls.def("set_radius", &Class::setRadius);
    cls.def("set_threshold", &Class::setThreshold);
    cls.def("set_non_max_supression", &Class::setNonMaxSupression);
    cls.def("set_refine", &Class::setRefine);
    cls.def("set_normals", &Class::setNormals);
    cls.def("set_search_surface", &Class::setSearchSurface);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineKeypointsHarris3dClasses(py::module &sub_module) {
}