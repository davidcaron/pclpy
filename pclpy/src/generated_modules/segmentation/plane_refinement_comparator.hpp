
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/plane_refinement_comparator.h>



template<typename PointT, typename PointNT, typename PointLT>
void defineSegmentationPlaneRefinementComparator(py::module &m, std::string const & suffix) {
    using Class = PlaneRefinementComparator<PointT, PointNT, PointLT>;
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
    py::class_<Class, PlaneCoefficientComparator<PointT,PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<boost::shared_ptr<std::vector<pcl::ModelCoefficients> >, boost::shared_ptr<std::vector<bool> >>(), "models"_a, "refine_labels"_a);
    cls.def("set_labels", &Class::setLabels);
    cls.def("compare", &Class::compare);
    cls.def("set_model_coefficients", py::overload_cast<boost::shared_ptr<std::vector<pcl::ModelCoefficients> > &> (&Class::setModelCoefficients));
    cls.def("set_model_coefficients", py::overload_cast<std::vector<pcl::ModelCoefficients> &> (&Class::setModelCoefficients));
    cls.def("set_refine_labels", py::overload_cast<boost::shared_ptr<std::vector<bool> > &> (&Class::setRefineLabels));
    cls.def("set_refine_labels", py::overload_cast<std::vector<bool> &> (&Class::setRefineLabels));
    cls.def("set_label_to_model", py::overload_cast<boost::shared_ptr<std::vector<int> > &> (&Class::setLabelToModel));
    cls.def("set_label_to_model", py::overload_cast<std::vector<int> &> (&Class::setLabelToModel));
        
}

void defineSegmentationPlaneRefinementComparatorClasses(py::module &sub_module) {
}