
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

#include <pcl/features/3dsc.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::ShapeContext1980>
void defineFeaturesShapeContext3DEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::ShapeContext3DEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "random"_a=false);
    cls.def("setMinimalRadius", &Class::setMinimalRadius, "radius"_a);
    cls.def("setPointDensityRadius", &Class::setPointDensityRadius, "radius"_a);
    cls.def("getAzimuthBins", &Class::getAzimuthBins);
    cls.def("getElevationBins", &Class::getElevationBins);
    cls.def("getRadiusBins", &Class::getRadiusBins);
    cls.def("getMinimalRadius", &Class::getMinimalRadius);
    cls.def("getPointDensityRadius", &Class::getPointDensityRadius);
        
}

void defineFeatures3dscFunctions(py::module &m) {
}

void defineFeatures3dscClasses(py::module &sub_module) {
    py::module sub_module_ShapeContext3DEstimation = sub_module.def_submodule("ShapeContext3DEstimation", "Submodule ShapeContext3DEstimation");
    defineFeaturesShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980>(sub_module_ShapeContext3DEstimation, "PointXYZ_Normal_ShapeContext1980");
    defineFeaturesShapeContext3DEstimation<pcl::PointXYZI, pcl::Normal, pcl::ShapeContext1980>(sub_module_ShapeContext3DEstimation, "PointXYZI_Normal_ShapeContext1980");
    defineFeaturesShapeContext3DEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::ShapeContext1980>(sub_module_ShapeContext3DEstimation, "PointXYZRGBA_Normal_ShapeContext1980");
}