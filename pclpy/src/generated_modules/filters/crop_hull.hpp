
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/filters/crop_hull.h>



template<typename PointT>
void defineFiltersCropHull(py::module &m, std::string const & suffix) {
    using Class = pcl::CropHull<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setHullIndices", &Class::setHullIndices, "polygons"_a);
    cls.def("setHullCloud", &Class::setHullCloud, "points"_a);
    cls.def("setDim", &Class::setDim, "dim"_a);
    cls.def("setCropOutside", &Class::setCropOutside, "crop_outside"_a);
    cls.def("getHullIndices", &Class::getHullIndices);
    cls.def("getHullCloud", &Class::getHullCloud);
        
}

void defineFiltersCropHullFunctions(py::module &m) {
}

void defineFiltersCropHullClasses(py::module &sub_module) {
    py::module sub_module_CropHull = sub_module.def_submodule("CropHull", "Submodule CropHull");
    defineFiltersCropHull<pcl::InterestPoint>(sub_module_CropHull, "InterestPoint");
    defineFiltersCropHull<pcl::PointDEM>(sub_module_CropHull, "PointDEM");
    defineFiltersCropHull<pcl::PointNormal>(sub_module_CropHull, "PointNormal");
    defineFiltersCropHull<pcl::PointSurfel>(sub_module_CropHull, "PointSurfel");
    defineFiltersCropHull<pcl::PointWithRange>(sub_module_CropHull, "PointWithRange");
    defineFiltersCropHull<pcl::PointWithScale>(sub_module_CropHull, "PointWithScale");
    defineFiltersCropHull<pcl::PointWithViewpoint>(sub_module_CropHull, "PointWithViewpoint");
    defineFiltersCropHull<pcl::PointXYZ>(sub_module_CropHull, "PointXYZ");
    defineFiltersCropHull<pcl::PointXYZHSV>(sub_module_CropHull, "PointXYZHSV");
    defineFiltersCropHull<pcl::PointXYZI>(sub_module_CropHull, "PointXYZI");
    defineFiltersCropHull<pcl::PointXYZINormal>(sub_module_CropHull, "PointXYZINormal");
    defineFiltersCropHull<pcl::PointXYZL>(sub_module_CropHull, "PointXYZL");
    defineFiltersCropHull<pcl::PointXYZLNormal>(sub_module_CropHull, "PointXYZLNormal");
    defineFiltersCropHull<pcl::PointXYZRGB>(sub_module_CropHull, "PointXYZRGB");
    defineFiltersCropHull<pcl::PointXYZRGBA>(sub_module_CropHull, "PointXYZRGBA");
    defineFiltersCropHull<pcl::PointXYZRGBL>(sub_module_CropHull, "PointXYZRGBL");
    defineFiltersCropHull<pcl::PointXYZRGBNormal>(sub_module_CropHull, "PointXYZRGBNormal");
}