
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/feature_histogram.h>



void defineCommonFeatureHistogram(py::module &m) {
    using Class = FeatureHistogram;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FeatureHistogram");
    cls.def(py::init<size_t, float, float>(), "number_of_bins"_a, "min"_a, "max"_a);
    cls.def("add_value", &Class::addValue);
}

void defineCommonFeatureHistogramClasses(py::module &sub_module) {
    defineCommonFeatureHistogram(sub_module);
}