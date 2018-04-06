
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/crop_hull.h>



template<typename PointT>
void defineFiltersCropHull(py::module &m, std::string const & suffix) {
    using Class = CropHull<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("hull_indices", &Class::getHullIndices, &Class::setHullIndices);
    cls.def_property("hull_cloud", &Class::getHullCloud, &Class::setHullCloud);
    cls.def("set_dim", &Class::setDim);
    cls.def("set_crop_outside", &Class::setCropOutside);
        
}

void defineFiltersCropHullClasses(py::module &sub_module) {
    py::module sub_module_CropHull = sub_module.def_submodule("CropHull", "Submodule CropHull");
    defineFiltersCropHull<InterestPoint>(sub_module_CropHull, "InterestPoint");
    defineFiltersCropHull<PointDEM>(sub_module_CropHull, "PointDEM");
    defineFiltersCropHull<PointNormal>(sub_module_CropHull, "PointNormal");
    defineFiltersCropHull<PointSurfel>(sub_module_CropHull, "PointSurfel");
    defineFiltersCropHull<PointWithRange>(sub_module_CropHull, "PointWithRange");
    defineFiltersCropHull<PointWithScale>(sub_module_CropHull, "PointWithScale");
    defineFiltersCropHull<PointWithViewpoint>(sub_module_CropHull, "PointWithViewpoint");
    defineFiltersCropHull<PointXYZ>(sub_module_CropHull, "PointXYZ");
    defineFiltersCropHull<PointXYZHSV>(sub_module_CropHull, "PointXYZHSV");
    defineFiltersCropHull<PointXYZI>(sub_module_CropHull, "PointXYZI");
    defineFiltersCropHull<PointXYZINormal>(sub_module_CropHull, "PointXYZINormal");
    defineFiltersCropHull<PointXYZL>(sub_module_CropHull, "PointXYZL");
    defineFiltersCropHull<PointXYZLNormal>(sub_module_CropHull, "PointXYZLNormal");
    defineFiltersCropHull<PointXYZRGB>(sub_module_CropHull, "PointXYZRGB");
    defineFiltersCropHull<PointXYZRGBA>(sub_module_CropHull, "PointXYZRGBA");
    defineFiltersCropHull<PointXYZRGBL>(sub_module_CropHull, "PointXYZRGBL");
    defineFiltersCropHull<PointXYZRGBNormal>(sub_module_CropHull, "PointXYZRGBNormal");
}