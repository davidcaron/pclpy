
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/crop_box.h>



template<typename PointT>
void defineFiltersCropBox(py::module &m, std::string const & suffix) {
    using Class = CropBox<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def_property("min", &Class::getMin, &Class::setMin);
    cls.def_property("max", &Class::getMax, &Class::setMax);
    cls.def_property("translation", &Class::getTranslation, &Class::setTranslation);
    cls.def_property("rotation", &Class::getRotation, &Class::setRotation);
    cls.def_property("transform", &Class::getTransform, &Class::setTransform);
        
}



void defineFiltersCropBoxClasses(py::module &sub_module) {
    py::module sub_module_CropBox = sub_module.def_submodule("CropBox", "Submodule CropBox");
    defineFiltersCropBox<InterestPoint>(sub_module_CropBox, "InterestPoint");
    defineFiltersCropBox<PointDEM>(sub_module_CropBox, "PointDEM");
    defineFiltersCropBox<PointNormal>(sub_module_CropBox, "PointNormal");
    defineFiltersCropBox<PointSurfel>(sub_module_CropBox, "PointSurfel");
    defineFiltersCropBox<PointWithRange>(sub_module_CropBox, "PointWithRange");
    defineFiltersCropBox<PointWithScale>(sub_module_CropBox, "PointWithScale");
    defineFiltersCropBox<PointWithViewpoint>(sub_module_CropBox, "PointWithViewpoint");
    defineFiltersCropBox<PointXYZ>(sub_module_CropBox, "PointXYZ");
    defineFiltersCropBox<PointXYZHSV>(sub_module_CropBox, "PointXYZHSV");
    defineFiltersCropBox<PointXYZI>(sub_module_CropBox, "PointXYZI");
    defineFiltersCropBox<PointXYZINormal>(sub_module_CropBox, "PointXYZINormal");
    defineFiltersCropBox<PointXYZL>(sub_module_CropBox, "PointXYZL");
    defineFiltersCropBox<PointXYZLNormal>(sub_module_CropBox, "PointXYZLNormal");
    defineFiltersCropBox<PointXYZRGB>(sub_module_CropBox, "PointXYZRGB");
    defineFiltersCropBox<PointXYZRGBA>(sub_module_CropBox, "PointXYZRGBA");
    defineFiltersCropBox<PointXYZRGBL>(sub_module_CropBox, "PointXYZRGBL");
    defineFiltersCropBox<PointXYZRGBNormal>(sub_module_CropBox, "PointXYZRGBNormal");
}