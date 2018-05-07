
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/comparator.h>



template <typename PointT>
void defineSegmentationComparator(py::module &m, std::string const & suffix) {
    using Class = pcl::Comparator<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("compare", &Class::compare, "idx1"_a, "idx2"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("getInputCloud", &Class::getInputCloud);
        
}

void defineSegmentationComparatorFunctions(py::module &m) {
}

void defineSegmentationComparatorClasses(py::module &sub_module) {
}