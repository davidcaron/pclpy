
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
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/plane_refinement_comparator.h>



template<typename PointT, typename PointNT, typename PointLT>
void defineSegmentationPlaneRefinementComparator(py::module &m, std::string const & suffix) {
    using Class = pcl::PlaneRefinementComparator<PointT, PointNT, PointLT>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using PointCloudL = Class::PointCloudL;
    using PointCloudLPtr = Class::PointCloudLPtr;
    using PointCloudLConstPtr = Class::PointCloudLConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::PlaneCoefficientComparator<PointT, PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<boost::shared_ptr<std::vector<pcl::ModelCoefficients> >, boost::shared_ptr<std::vector<bool> >>(), "models"_a, "refine_labels"_a);
    cls.def("compare", &Class::compare, "idx1"_a, "idx2"_a);
    cls.def("setModelCoefficients", py::overload_cast<boost::shared_ptr<std::vector<pcl::ModelCoefficients> > &> (&Class::setModelCoefficients), "models"_a);
    cls.def("setModelCoefficients", py::overload_cast<std::vector<pcl::ModelCoefficients> &> (&Class::setModelCoefficients), "models"_a);
    cls.def("setRefineLabels", py::overload_cast<boost::shared_ptr<std::vector<bool> > &> (&Class::setRefineLabels), "refine_labels"_a);
    cls.def("setRefineLabels", py::overload_cast<std::vector<bool> &> (&Class::setRefineLabels), "refine_labels"_a);
    cls.def("setLabelToModel", py::overload_cast<boost::shared_ptr<std::vector<int> > &> (&Class::setLabelToModel), "label_to_model"_a);
    cls.def("setLabelToModel", py::overload_cast<std::vector<int> &> (&Class::setLabelToModel), "label_to_model"_a);
    cls.def("setLabels", &Class::setLabels, "labels"_a);
    cls.def("getModelCoefficients", &Class::getModelCoefficients);
        
}

void defineSegmentationPlaneRefinementComparatorFunctions(py::module &m) {
}

void defineSegmentationPlaneRefinementComparatorClasses(py::module &sub_module) {
}