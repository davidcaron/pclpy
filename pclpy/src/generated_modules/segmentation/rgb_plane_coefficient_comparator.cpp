
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

#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>



template<typename PointT, typename PointNT>
void defineSegmentationRGBPlaneCoefficientComparator(py::module &m, std::string const & suffix) {
    using Class = pcl::RGBPlaneCoefficientComparator<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::PlaneCoefficientComparator<PointT, PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<boost::shared_ptr<std::vector<float> >>(), "plane_coeff_d"_a);
    cls.def("compare", &Class::compare, "idx1"_a, "idx2"_a);
    cls.def("setColorThreshold", &Class::setColorThreshold, "color_threshold"_a);
    cls.def("getColorThreshold", &Class::getColorThreshold);
        
}

void defineSegmentationRgbPlaneCoefficientComparatorFunctions(py::module &m) {
}

void defineSegmentationRgbPlaneCoefficientComparatorClasses(py::module &sub_module) {
}