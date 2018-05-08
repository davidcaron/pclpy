
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

#include <pcl/segmentation/progressive_morphological_filter.h>



template <typename PointT>
void defineSegmentationProgressiveMorphologicalFilter(py::module &m, std::string const & suffix) {
    using Class = pcl::ProgressiveMorphologicalFilter<PointT>;
    using PointCloud = Class::PointCloud;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("extract", &Class::extract, "ground"_a);
    cls.def("setMaxWindowSize", &Class::setMaxWindowSize, "max_window_size"_a);
    cls.def("setSlope", &Class::setSlope, "slope"_a);
    cls.def("setMaxDistance", &Class::setMaxDistance, "max_distance"_a);
    cls.def("setInitialDistance", &Class::setInitialDistance, "initial_distance"_a);
    cls.def("setCellSize", &Class::setCellSize, "cell_size"_a);
    cls.def("setBase", &Class::setBase, "base"_a);
    cls.def("setExponential", &Class::setExponential, "exponential"_a);
    cls.def("getMaxWindowSize", &Class::getMaxWindowSize);
    cls.def("getSlope", &Class::getSlope);
    cls.def("getMaxDistance", &Class::getMaxDistance);
    cls.def("getInitialDistance", &Class::getInitialDistance);
    cls.def("getCellSize", &Class::getCellSize);
    cls.def("getBase", &Class::getBase);
    cls.def("getExponential", &Class::getExponential);
        
}

void defineSegmentationProgressiveMorphologicalFilterFunctions(py::module &m) {
}

void defineSegmentationProgressiveMorphologicalFilterClasses(py::module &sub_module) {
    py::module sub_module_ProgressiveMorphologicalFilter = sub_module.def_submodule("ProgressiveMorphologicalFilter", "Submodule ProgressiveMorphologicalFilter");
    defineSegmentationProgressiveMorphologicalFilter<pcl::PointXYZ>(sub_module_ProgressiveMorphologicalFilter, "PointXYZ");
    defineSegmentationProgressiveMorphologicalFilter<pcl::PointXYZI>(sub_module_ProgressiveMorphologicalFilter, "PointXYZI");
    defineSegmentationProgressiveMorphologicalFilter<pcl::PointXYZRGB>(sub_module_ProgressiveMorphologicalFilter, "PointXYZRGB");
    defineSegmentationProgressiveMorphologicalFilter<pcl::PointXYZRGBA>(sub_module_ProgressiveMorphologicalFilter, "PointXYZRGBA");
}