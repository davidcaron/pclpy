
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/colors.h>



void defineCommonGlasbeyLUT(py::module &m) {
    using Class = GlasbeyLUT;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "GlasbeyLUT");
    cls.def("at", &Class::at);
    cls.def("size", &Class::size);
    cls.def("data", &Class::data);
}

void defineCommonColorsClasses(py::module &sub_module) {
    defineCommonGlasbeyLUT(sub_module);
}