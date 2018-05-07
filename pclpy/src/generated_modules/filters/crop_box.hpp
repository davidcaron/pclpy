
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/filters/crop_box.h>



template<typename PointT>
void defineFiltersCropBox(py::module &m, std::string const & suffix) {
    using Class = pcl::CropBox<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def("setMin", &Class::setMin, "min_pt"_a);
    cls.def("setMax", &Class::setMax, "max_pt"_a);
    cls.def("setTranslation", &Class::setTranslation, "translation"_a);
    cls.def("setRotation", &Class::setRotation, "rotation"_a);
    cls.def("setTransform", &Class::setTransform, "transform"_a);
    cls.def("getMin", &Class::getMin);
    cls.def("getMax", &Class::getMax);
    cls.def("getTranslation", &Class::getTranslation);
    cls.def("getRotation", &Class::getRotation);
    cls.def("getTransform", &Class::getTransform);
        
}

void defineFiltersCropBoxFunctions(py::module &m) {
}

void defineFiltersCropBoxClasses(py::module &sub_module) {
    py::module sub_module_CropBox = sub_module.def_submodule("CropBox", "Submodule CropBox");
    defineFiltersCropBox<pcl::InterestPoint>(sub_module_CropBox, "InterestPoint");
    defineFiltersCropBox<pcl::PointDEM>(sub_module_CropBox, "PointDEM");
    defineFiltersCropBox<pcl::PointNormal>(sub_module_CropBox, "PointNormal");
    defineFiltersCropBox<pcl::PointSurfel>(sub_module_CropBox, "PointSurfel");
    defineFiltersCropBox<pcl::PointWithRange>(sub_module_CropBox, "PointWithRange");
    defineFiltersCropBox<pcl::PointWithScale>(sub_module_CropBox, "PointWithScale");
    defineFiltersCropBox<pcl::PointWithViewpoint>(sub_module_CropBox, "PointWithViewpoint");
    defineFiltersCropBox<pcl::PointXYZ>(sub_module_CropBox, "PointXYZ");
    defineFiltersCropBox<pcl::PointXYZHSV>(sub_module_CropBox, "PointXYZHSV");
    defineFiltersCropBox<pcl::PointXYZI>(sub_module_CropBox, "PointXYZI");
    defineFiltersCropBox<pcl::PointXYZINormal>(sub_module_CropBox, "PointXYZINormal");
    defineFiltersCropBox<pcl::PointXYZL>(sub_module_CropBox, "PointXYZL");
    defineFiltersCropBox<pcl::PointXYZLNormal>(sub_module_CropBox, "PointXYZLNormal");
    defineFiltersCropBox<pcl::PointXYZRGB>(sub_module_CropBox, "PointXYZRGB");
    defineFiltersCropBox<pcl::PointXYZRGBA>(sub_module_CropBox, "PointXYZRGBA");
    defineFiltersCropBox<pcl::PointXYZRGBL>(sub_module_CropBox, "PointXYZRGBL");
    defineFiltersCropBox<pcl::PointXYZRGBNormal>(sub_module_CropBox, "PointXYZRGBNormal");
}