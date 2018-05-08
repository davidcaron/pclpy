
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

#include <pcl/visualization/common/float_image_utils.h>

using namespace pcl::visualization;


void defineVisualizationFloatImageUtils(py::module &m) {
    using Class = pcl::visualization::FloatImageUtils;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FloatImageUtils");
    cls.def_static("getColorForFloat", &Class::getColorForFloat, "value"_a, "r"_a, "g"_a, "b"_a);
    cls.def_static("getColorForAngle", &Class::getColorForAngle, "value"_a, "r"_a, "g"_a, "b"_a);
    cls.def_static("getColorForHalfAngle", &Class::getColorForHalfAngle, "value"_a, "r"_a, "g"_a, "b"_a);
    cls.def_static("getVisualImage", py::overload_cast<const float *, int, int, float, float, bool> (&Class::getVisualImage), "float_image"_a, "width"_a, "height"_a, "min_value"_a=-std::numeric_limits<float>::infinity(), "max_value"_a=std::numeric_limits<float>::infinity(), "gray_scale"_a=false);
    cls.def_static("getVisualImage", py::overload_cast<const unsigned short *, int, int, unsigned short, unsigned short, bool> (&Class::getVisualImage), "float_image"_a, "width"_a, "height"_a, "min_value"_a=0, "max_value"_a=std::numeric_limits<unsigned short>::max(), "gray_scale"_a=false);
    cls.def_static("getVisualAngleImage", &Class::getVisualAngleImage, "angle_image"_a, "width"_a, "height"_a);
    cls.def_static("getVisualHalfAngleImage", &Class::getVisualHalfAngleImage, "angle_image"_a, "width"_a, "height"_a);
}

void defineVisualizationFloatImageUtilsFunctions(py::module &m) {
}

void defineVisualizationFloatImageUtilsClasses(py::module &sub_module) {
    defineVisualizationFloatImageUtils(sub_module);
}