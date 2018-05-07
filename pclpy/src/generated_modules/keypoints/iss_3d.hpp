
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/keypoints/iss_3d.h>



template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
void defineKeypointsISSKeypoint3D(py::module &m, std::string const & suffix) {
    using Class = pcl::ISSKeypoint3D<PointInT, PointOutT, NormalT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using OctreeSearchIn = Class::OctreeSearchIn;
    using OctreeSearchInPtr = Class::OctreeSearchInPtr;
    py::class_<Class, pcl::Keypoint<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "salient_radius"_a=0.0001);
    cls.def("setSalientRadius", &Class::setSalientRadius, "salient_radius"_a);
    cls.def("setNonMaxRadius", &Class::setNonMaxRadius, "non_max_radius"_a);
    cls.def("setNormalRadius", &Class::setNormalRadius, "normal_radius"_a);
    cls.def("setBorderRadius", &Class::setBorderRadius, "border_radius"_a);
    cls.def("setThreshold21", &Class::setThreshold21, "gamma_21"_a);
    cls.def("setThreshold32", &Class::setThreshold32, "gamma_32"_a);
    cls.def("setMinNeighbors", &Class::setMinNeighbors, "min_neighbors"_a);
    cls.def("setNormals", &Class::setNormals, "normals"_a);
    cls.def("setAngleThreshold", &Class::setAngleThreshold, "angle"_a);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineKeypointsIss3dFunctions(py::module &m) {
}

void defineKeypointsIss3dClasses(py::module &sub_module) {
}