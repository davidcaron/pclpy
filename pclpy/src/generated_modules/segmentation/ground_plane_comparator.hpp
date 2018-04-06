
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/ground_plane_comparator.h>



template<typename PointT, typename PointNT>
void defineSegmentationGroundPlaneComparator(py::module &m, std::string const & suffix) {
    using Class = GroundPlaneComparator<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Comparator<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<boost::shared_ptr<std::vector<float> >>(), "plane_coeff_d"_a);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def_property("input_normals", &Class::getInputNormals, &Class::setInputNormals);
    cls.def_property("angular_threshold", &Class::getAngularThreshold, &Class::setAngularThreshold);
    cls.def("set_ground_angular_threshold", &Class::setGroundAngularThreshold);
    cls.def("set_expected_ground_normal", &Class::setExpectedGroundNormal);
    cls.def_property("distance_threshold", &Class::getDistanceThreshold, &Class::setDistanceThreshold);
    cls.def("compare", &Class::compare);
    cls.def("set_plane_coeff_d", py::overload_cast<boost::shared_ptr<std::vector<float> > &> (&Class::setPlaneCoeffD));
    cls.def("set_plane_coeff_d", py::overload_cast<std::vector<float> &> (&Class::setPlaneCoeffD));
        
}

void defineSegmentationGroundPlaneComparatorClasses(py::module &sub_module) {
}