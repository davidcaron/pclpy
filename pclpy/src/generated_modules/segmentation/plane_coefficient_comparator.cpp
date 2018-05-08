
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

#include <pcl/segmentation/plane_coefficient_comparator.h>



template<typename PointT, typename PointNT>
void defineSegmentationPlaneCoefficientComparator(py::module &m, std::string const & suffix) {
    using Class = pcl::PlaneCoefficientComparator<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Comparator<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<boost::shared_ptr<std::vector<float> >>(), "plane_coeff_d"_a);
    cls.def("compare", &Class::compare, "idx1"_a, "idx2"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setInputNormals", &Class::setInputNormals, "normals"_a);
    cls.def("setPlaneCoeffD", py::overload_cast<boost::shared_ptr<std::vector<float> > &> (&Class::setPlaneCoeffD), "plane_coeff_d"_a);
    cls.def("setPlaneCoeffD", py::overload_cast<std::vector<float> &> (&Class::setPlaneCoeffD), "plane_coeff_d"_a);
    cls.def("setAngularThreshold", &Class::setAngularThreshold, "angular_threshold"_a);
    cls.def("setDistanceThreshold", &Class::setDistanceThreshold, "distance_threshold"_a, "depth_dependent"_a=false);
    cls.def("getInputNormals", &Class::getInputNormals);
    cls.def("getPlaneCoeffD", &Class::getPlaneCoeffD);
    cls.def("getAngularThreshold", &Class::getAngularThreshold);
    cls.def("getDistanceThreshold", &Class::getDistanceThreshold);
        
}

void defineSegmentationPlaneCoefficientComparatorFunctions(py::module &m) {
}

void defineSegmentationPlaneCoefficientComparatorClasses(py::module &sub_module) {
}