
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>



template<typename PointT, typename PointNT>
void defineSegmentationEuclideanPlaneCoefficientComparator(py::module &m, std::string const & suffix) {
    using Class = EuclideanPlaneCoefficientComparator<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PlaneCoefficientComparator<PointT,PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compare", &Class::compare);
        
}

void defineSegmentationEuclideanPlaneCoefficientComparatorClasses(py::module &sub_module) {
}