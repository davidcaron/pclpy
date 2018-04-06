
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/debayer.h>

using namespace pcl::io;


void defineIoDeBayer(py::module &m) {
    using Class = io::DeBayer;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DeBayer");
    cls.def("debayer_bilinear", &Class::debayerBilinear);
    cls.def("debayer_edge_aware", &Class::debayerEdgeAware);
    cls.def("debayer_edge_aware_weighted", &Class::debayerEdgeAwareWeighted);
}

void defineIoDebayerClasses(py::module &sub_module) {
    defineIoDeBayer(sub_module);
}