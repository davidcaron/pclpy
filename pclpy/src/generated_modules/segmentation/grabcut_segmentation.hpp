
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/grabcut_segmentation.h>



template <typename PointT>
void defineSegmentationGrabCut(py::module &m, std::string const & suffix) {
    using Class = pcl::GrabCut<PointT>;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<uint32_t, float>(), "K"_a=5, "lambda"_a=50f);
    cls.def("refine", &Class::refine);
    cls.def("refineOnce", &Class::refineOnce);
    cls.def("extract", &Class::extract, "clusters"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setBackgroundPoints", &Class::setBackgroundPoints, "background_points"_a);
    cls.def("setBackgroundPointsIndices", py::overload_cast<int, int, int, int> (&Class::setBackgroundPointsIndices), "x1"_a, "y1"_a, "x2"_a, "y2"_a);
    cls.def("setBackgroundPointsIndices", py::overload_cast<const Class::Pointpcl::IndicesConstPtr &> (&Class::setBackgroundPointsIndices), "indices"_a);
    cls.def("setLambda", &Class::setLambda, "lambda"_a);
    cls.def("setK", &Class::setK, "K"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("setNumberOfNeighbours", &Class::setNumberOfNeighbours, "nb_neighbours"_a);
    cls.def("getLambda", &Class::getLambda);
    cls.def("getK", &Class::getK);
    cls.def("getSearchMethod", &Class::getSearchMethod);
    cls.def("getNumberOfNeighbours", &Class::getNumberOfNeighbours);
        
}

void defineSegmentationGrabcutSegmentationFunctions(py::module &m) {
}

void defineSegmentationGrabcutSegmentationClasses(py::module &sub_module) {
    py::enum_<pcl::segmentation::grabcut::SegmentationValue>(sub_module, "SegmentationValue")
        .value("SegmentationForeground", pcl::segmentation::grabcut::SegmentationValue::SegmentationForeground)
        .value("SegmentationBackground", pcl::segmentation::grabcut::SegmentationValue::SegmentationBackground)
        .export_values();
    py::enum_<pcl::segmentation::grabcut::TrimapValue>(sub_module, "TrimapValue")
        .value("TrimapUnknown", pcl::segmentation::grabcut::TrimapValue::TrimapUnknown)
        .value("TrimapForeground", pcl::segmentation::grabcut::TrimapValue::TrimapForeground)
        .value("TrimapBackground", pcl::segmentation::grabcut::TrimapValue::TrimapBackground)
        .export_values();
}