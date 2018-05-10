
#include <pcl/common/feature_histogram.h>



void defineCommonFeatureHistogram(py::module &m) {
    using Class = pcl::FeatureHistogram;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FeatureHistogram");
    cls.def(py::init<size_t, float, float>(), "number_of_bins"_a, "min"_a, "max"_a);
    cls.def("addValue", &Class::addValue, "value"_a);
    cls.def("getThresholdMin", &Class::getThresholdMin);
    cls.def("getThresholdMax", &Class::getThresholdMax);
    cls.def("getNumberOfElements", &Class::getNumberOfElements);
    cls.def("getNumberOfBins", &Class::getNumberOfBins);
    cls.def("getMeanValue", &Class::getMeanValue);
    cls.def("getVariance", &Class::getVariance, "mean"_a);
}

void defineCommonFeatureHistogramFunctions(py::module &m) {
}

void defineCommonFeatureHistogramClasses(py::module &sub_module) {
    defineCommonFeatureHistogram(sub_module);
}