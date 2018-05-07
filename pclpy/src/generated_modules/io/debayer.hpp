
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/io/debayer.h>

using namespace pcl::io;


void defineIoDeBayer(py::module &m) {
    using Class = pcl::io::DeBayer;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DeBayer");
    cls.def("debayerBilinear", &Class::debayerBilinear, "bayer_pixel"_a, "rgb_buffer"_a, "width"_a, "height"_a, "bayer_line_step"_a=0, "bayer_line_step2"_a=0, "rgb_line_step"_a=0);
    cls.def("debayerEdgeAware", &Class::debayerEdgeAware, "bayer_pixel"_a, "rgb_buffer"_a, "width"_a, "height"_a, "bayer_line_step"_a=0, "bayer_line_step2"_a=0, "rgb_line_step"_a=0);
    cls.def("debayerEdgeAwareWeighted", &Class::debayerEdgeAwareWeighted, "bayer_pixel"_a, "rgb_buffer"_a, "width"_a, "height"_a, "bayer_line_step"_a=0, "bayer_line_step2"_a=0, "rgb_line_step"_a=0);
}

void defineIoDebayerFunctions(py::module &m) {
}

void defineIoDebayerClasses(py::module &sub_module) {
    defineIoDeBayer(sub_module);
}