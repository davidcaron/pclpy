
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

#include <pcl/keypoints/harris_3d.h>



template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
void defineKeypointsHarrisKeypoint3D(py::module &m, std::string const & suffix) {
    using Class = pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    py::class_<Class, pcl::Keypoint<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::ResponseMethod>(cls, "ResponseMethod")
        .value("HARRIS", Class::ResponseMethod::HARRIS)
        .value("NOBLE", Class::ResponseMethod::NOBLE)
        .value("LOWE", Class::ResponseMethod::LOWE)
        .value("TOMASI", Class::ResponseMethod::TOMASI)
        .value("CURVATURE", Class::ResponseMethod::CURVATURE)
        .export_values();
    cls.def(py::init<Class::ResponseMethod, float, float>(), "method"_a=Class::HARRIS, "radius"_a=0.01f, "threshold"_a=0.0f);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setMethod", &Class::setMethod, "type"_a);
    cls.def("setRadius", &Class::setRadius, "radius"_a);
    cls.def("setThreshold", &Class::setThreshold, "threshold"_a);
    cls.def("setNonMaxSupression", &Class::setNonMaxSupression, "bool"_a=false);
    cls.def("setRefine", &Class::setRefine, "do_refine"_a);
    cls.def("setNormals", &Class::setNormals, "normals"_a);
    cls.def("setSearchSurface", &Class::setSearchSurface, "cloud"_a);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineKeypointsHarris3dFunctions(py::module &m) {
}

void defineKeypointsHarris3dClasses(py::module &sub_module) {
}