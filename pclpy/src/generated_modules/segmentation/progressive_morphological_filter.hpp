
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/progressive_morphological_filter.h>



template <typename PointT>
void defineSegmentationProgressiveMorphologicalFilter(py::module &m, std::string const & suffix) {
    using Class = ProgressiveMorphologicalFilter<PointT>;
    using PointCloud = Class::PointCloud;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("max_window_size", &Class::getMaxWindowSize, &Class::setMaxWindowSize);
    cls.def_property("slope", &Class::getSlope, &Class::setSlope);
    cls.def_property("max_distance", &Class::getMaxDistance, &Class::setMaxDistance);
    cls.def_property("initial_distance", &Class::getInitialDistance, &Class::setInitialDistance);
    cls.def_property("cell_size", &Class::getCellSize, &Class::setCellSize);
    cls.def_property("base", &Class::getBase, &Class::setBase);
    cls.def_property("exponential", &Class::getExponential, &Class::setExponential);
    cls.def("extract", &Class::extract);
        
}

void defineSegmentationProgressiveMorphologicalFilterClasses(py::module &sub_module) {
    py::module sub_module_ProgressiveMorphologicalFilter = sub_module.def_submodule("ProgressiveMorphologicalFilter", "Submodule ProgressiveMorphologicalFilter");
    defineSegmentationProgressiveMorphologicalFilter<PointXYZ>(sub_module_ProgressiveMorphologicalFilter, "PointXYZ");
    defineSegmentationProgressiveMorphologicalFilter<PointXYZI>(sub_module_ProgressiveMorphologicalFilter, "PointXYZI");
    defineSegmentationProgressiveMorphologicalFilter<PointXYZRGB>(sub_module_ProgressiveMorphologicalFilter, "PointXYZRGB");
    defineSegmentationProgressiveMorphologicalFilter<PointXYZRGBA>(sub_module_ProgressiveMorphologicalFilter, "PointXYZRGBA");
}