
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>



template <typename PointT>
void defineSegmentationApproximateProgressiveMorphologicalFilter(py::module &m, std::string const & suffix) {
    using Class = pcl::ApproximateProgressiveMorphologicalFilter<PointT>;
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
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
    cls.def("getMaxWindowSize", &Class::getMaxWindowSize);
    cls.def("getSlope", &Class::getSlope);
    cls.def("getMaxDistance", &Class::getMaxDistance);
    cls.def("getInitialDistance", &Class::getInitialDistance);
    cls.def("getCellSize", &Class::getCellSize);
    cls.def("getBase", &Class::getBase);
    cls.def("getExponential", &Class::getExponential);
        
}

void defineSegmentationApproximateProgressiveMorphologicalFilterFunctions(py::module &m) {
}

void defineSegmentationApproximateProgressiveMorphologicalFilterClasses(py::module &sub_module) {
    py::module sub_module_ApproximateProgressiveMorphologicalFilter = sub_module.def_submodule("ApproximateProgressiveMorphologicalFilter", "Submodule ApproximateProgressiveMorphologicalFilter");
    defineSegmentationApproximateProgressiveMorphologicalFilter<pcl::PointXYZ>(sub_module_ApproximateProgressiveMorphologicalFilter, "PointXYZ");
    defineSegmentationApproximateProgressiveMorphologicalFilter<pcl::PointXYZI>(sub_module_ApproximateProgressiveMorphologicalFilter, "PointXYZI");
    defineSegmentationApproximateProgressiveMorphologicalFilter<pcl::PointXYZRGB>(sub_module_ApproximateProgressiveMorphologicalFilter, "PointXYZRGB");
    defineSegmentationApproximateProgressiveMorphologicalFilter<pcl::PointXYZRGBA>(sub_module_ApproximateProgressiveMorphologicalFilter, "PointXYZRGBA");
}