
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/stereo/digital_elevation_map.h>



void defineStereoDigitalElevationMapBuilder(py::module &m) {
    using Class = DigitalElevationMapBuilder;
    py::class_<Class, DisparityMapConverter<PointDEM>, boost::shared_ptr<Class>> cls(m, "DigitalElevationMapBuilder");
    cls.def(py::init<>());
    cls.def("set_resolution", &Class::setResolution);
    cls.def_property("min_points_in_cell", &Class::getMinPointsInCell, &Class::setMinPointsInCell);
    cls.def("compute", py::overload_cast<pcl::PointCloud<PointDEM> &> (&Class::compute));
}

void defineStereoDigitalElevationMapClasses(py::module &sub_module) {
    defineStereoDigitalElevationMapBuilder(sub_module);
}