
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/3dsc.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::ShapeContext1980>
void defineFeaturesShapeContext3DEstimation(py::module &m, std::string const & suffix) {
    using Class = ShapeContext3DEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "random"_a=false);
    cls.def_property("minimal_radius", &Class::getMinimalRadius, &Class::setMinimalRadius);
    cls.def_property("point_density_radius", &Class::getPointDensityRadius, &Class::setPointDensityRadius);
        
}

void defineFeatures3dscClasses(py::module &sub_module) {
    py::module sub_module_ShapeContext3DEstimation = sub_module.def_submodule("ShapeContext3DEstimation", "Submodule ShapeContext3DEstimation");
    defineFeaturesShapeContext3DEstimation<PointXYZ, Normal, ShapeContext1980>(sub_module_ShapeContext3DEstimation, "PointXYZ_Normal_ShapeContext1980");
    defineFeaturesShapeContext3DEstimation<PointXYZI, Normal, ShapeContext1980>(sub_module_ShapeContext3DEstimation, "PointXYZI_Normal_ShapeContext1980");
    defineFeaturesShapeContext3DEstimation<PointXYZRGBA, Normal, ShapeContext1980>(sub_module_ShapeContext3DEstimation, "PointXYZRGBA_Normal_ShapeContext1980");
}