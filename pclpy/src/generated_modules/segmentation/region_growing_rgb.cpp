
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/segmentation/region_growing_rgb.h>



template <typename PointT, typename NormalT = pcl::Normal>
void defineSegmentationRegionGrowingRGB(py::module &m, std::string const & suffix) {
    using Class = pcl::RegionGrowingRGB<PointT, NormalT>;
    py::class_<Class, pcl::RegionGrowing<PointT, NormalT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("extract", &Class::extract, "clusters"_a);
    cls.def("setPointColorThreshold", &Class::setPointColorThreshold, "thresh"_a);
    cls.def("setRegionColorThreshold", &Class::setRegionColorThreshold, "thresh"_a);
    cls.def("setDistanceThreshold", &Class::setDistanceThreshold, "thresh"_a);
    cls.def("setNumberOfRegionNeighbours", &Class::setNumberOfRegionNeighbours, "nghbr_number"_a);
    cls.def("setNormalTestFlag", &Class::setNormalTestFlag, "value"_a);
    cls.def("setCurvatureTestFlag", &Class::setCurvatureTestFlag, "value"_a);
    cls.def("setResidualTestFlag", &Class::setResidualTestFlag, "value"_a);
    cls.def("getPointColorThreshold", &Class::getPointColorThreshold);
    cls.def("getRegionColorThreshold", &Class::getRegionColorThreshold);
    cls.def("getDistanceThreshold", &Class::getDistanceThreshold);
    cls.def("getNumberOfRegionNeighbours", &Class::getNumberOfRegionNeighbours);
    cls.def("getNormalTestFlag", &Class::getNormalTestFlag);
    cls.def("getSegmentFromPoint", &Class::getSegmentFromPoint, "index"_a, "cluster"_a);
        
}

void defineSegmentationRegionGrowingRgbFunctions(py::module &m) {
}

void defineSegmentationRegionGrowingRgbClasses(py::module &sub_module) {
}