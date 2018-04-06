
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/keypoints/iss_3d.h>



template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
void defineKeypointsISSKeypoint3D(py::module &m, std::string const & suffix) {
    using Class = ISSKeypoint3D<PointInT, PointOutT, NormalT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using OctreeSearchIn = Class::OctreeSearchIn;
    using OctreeSearchInPtr = Class::OctreeSearchInPtr;
    py::class_<Class, Keypoint<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double>(), "salient_radius"_a=0.0001);
    cls.def("set_salient_radius", &Class::setSalientRadius);
    cls.def("set_non_max_radius", &Class::setNonMaxRadius);
    cls.def("set_normal_radius", &Class::setNormalRadius);
    cls.def("set_border_radius", &Class::setBorderRadius);
    cls.def("set_threshold21", &Class::setThreshold21);
    cls.def("set_threshold32", &Class::setThreshold32);
    cls.def("set_min_neighbors", &Class::setMinNeighbors);
    cls.def("set_normals", &Class::setNormals);
    cls.def("set_angle_threshold", &Class::setAngleThreshold);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineKeypointsIss3dClasses(py::module &sub_module) {
}