
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/region_growing_rgb.h>



template <typename PointT, typename NormalT = pcl::Normal>
void defineSegmentationRegionGrowingRGB(py::module &m, std::string const & suffix) {
    using Class = RegionGrowingRGB<PointT, NormalT>;
    py::class_<Class, RegionGrowing<PointT,NormalT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("point_color_threshold", &Class::getPointColorThreshold, &Class::setPointColorThreshold);
    cls.def_property("region_color_threshold", &Class::getRegionColorThreshold, &Class::setRegionColorThreshold);
    cls.def_property("distance_threshold", &Class::getDistanceThreshold, &Class::setDistanceThreshold);
    cls.def_property("number_of_region_neighbours", &Class::getNumberOfRegionNeighbours, &Class::setNumberOfRegionNeighbours);
    cls.def_property("normal_test_flag", &Class::getNormalTestFlag, &Class::setNormalTestFlag);
    cls.def("set_curvature_test_flag", &Class::setCurvatureTestFlag);
    cls.def("set_residual_test_flag", &Class::setResidualTestFlag);
    cls.def("extract", &Class::extract);
        
}

void defineSegmentationRegionGrowingRgbClasses(py::module &sub_module) {
}