
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/crf_segmentation.h>



template <typename PointT>
void defineSegmentationCrfSegmentation(py::module &m, std::string const & suffix) {
    using Class = pcl::CrfSegmentation<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("segmentPoints", &Class::segmentPoints, "output"_a);
    cls.def("createVoxelGrid", &Class::createVoxelGrid);
    cls.def("createDataVectorFromVoxelGrid", &Class::createDataVectorFromVoxelGrid);
    cls.def("createUnaryPotentials", &Class::createUnaryPotentials, "unary"_a, "colors"_a, "n_labels"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "input_cloud"_a);
    cls.def("setAnnotatedCloud", &Class::setAnnotatedCloud, "anno_cloud"_a);
    cls.def("setNormalCloud", &Class::setNormalCloud, "normal_cloud"_a);
    cls.def("setVoxelGridLeafSize", &Class::setVoxelGridLeafSize, "x"_a, "y"_a, "z"_a);
    cls.def("setNumberOfIterations", &Class::setNumberOfIterations, "n_iterations"_a=10);
    cls.def("setSmoothnessKernelParameters", &Class::setSmoothnessKernelParameters, "sx"_a, "sy"_a, "sz"_a, "w"_a);
    cls.def("setAppearanceKernelParameters", &Class::setAppearanceKernelParameters, "sx"_a, "sy"_a, "sz"_a, "sr"_a, "sg"_a, "sb"_a, "w"_a);
    cls.def("setSurfaceKernelParameters", &Class::setSurfaceKernelParameters, "sx"_a, "sy"_a, "sz"_a, "snx"_a, "sny"_a, "snz"_a, "w"_a);
        
}

void defineSegmentationCrfSegmentationFunctions(py::module &m) {
}

void defineSegmentationCrfSegmentationClasses(py::module &sub_module) {
}