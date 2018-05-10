
#include <pcl/stereo/digital_elevation_map.h>



void defineStereoDigitalElevationMapBuilder(py::module &m) {
    using Class = pcl::DigitalElevationMapBuilder;
    py::class_<Class, pcl::DisparityMapConverter<pcl::PointDEM>, boost::shared_ptr<Class>> cls(m, "DigitalElevationMapBuilder");
    cls.def(py::init<>());
    cls.def("compute", py::overload_cast<pcl::PointCloud<pcl::PointDEM> &> (&Class::compute), "out_cloud"_a);
    cls.def("setResolution", &Class::setResolution, "resolution_column"_a, "resolution_disparity"_a);
    cls.def("setMinPointsInCell", &Class::setMinPointsInCell, "min_points_in_cell"_a);
    cls.def("getColumnResolution", &Class::getColumnResolution);
    cls.def("getDisparityResolution", &Class::getDisparityResolution);
    cls.def("getMinPointsInCell", &Class::getMinPointsInCell);
}

void defineStereoDigitalElevationMapFunctions(py::module &m) {
}

void defineStereoDigitalElevationMapClasses(py::module &sub_module) {
    defineStereoDigitalElevationMapBuilder(sub_module);
}