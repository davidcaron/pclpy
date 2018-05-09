
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/filters/shadowpoints.h>



template<typename PointT, typename NormalT>
void defineFiltersShadowPoints(py::module &m, std::string const & suffix) {
    using Class = pcl::ShadowPoints<PointT, NormalT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def("setNormals", &Class::setNormals, "normals"_a);
    cls.def("setThreshold", &Class::setThreshold, "threshold"_a);
    cls.def("getNormals", &Class::getNormals);
    cls.def("getThreshold", &Class::getThreshold);
        
}

void defineFiltersShadowpointsFunctions(py::module &m) {
}

void defineFiltersShadowpointsClasses(py::module &sub_module) {
    py::module sub_module_ShadowPoints = sub_module.def_submodule("ShadowPoints", "Submodule ShadowPoints");
    defineFiltersShadowPoints<pcl::InterestPoint, pcl::Normal>(sub_module_ShadowPoints, "InterestPoint_Normal");
    defineFiltersShadowPoints<pcl::InterestPoint, pcl::PointNormal>(sub_module_ShadowPoints, "InterestPoint_PointNormal");
    defineFiltersShadowPoints<pcl::InterestPoint, pcl::PointSurfel>(sub_module_ShadowPoints, "InterestPoint_PointSurfel");
    defineFiltersShadowPoints<pcl::InterestPoint, pcl::PointXYZINormal>(sub_module_ShadowPoints, "InterestPoint_PointXYZINormal");
    defineFiltersShadowPoints<pcl::InterestPoint, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "InterestPoint_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::InterestPoint, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "InterestPoint_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointDEM, pcl::Normal>(sub_module_ShadowPoints, "PointDEM_Normal");
    defineFiltersShadowPoints<pcl::PointDEM, pcl::PointNormal>(sub_module_ShadowPoints, "PointDEM_PointNormal");
    defineFiltersShadowPoints<pcl::PointDEM, pcl::PointSurfel>(sub_module_ShadowPoints, "PointDEM_PointSurfel");
    defineFiltersShadowPoints<pcl::PointDEM, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointDEM_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointDEM, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointDEM_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointDEM, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointDEM_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointNormal, pcl::Normal>(sub_module_ShadowPoints, "PointNormal_Normal");
    defineFiltersShadowPoints<pcl::PointNormal, pcl::PointNormal>(sub_module_ShadowPoints, "PointNormal_PointNormal");
    defineFiltersShadowPoints<pcl::PointNormal, pcl::PointSurfel>(sub_module_ShadowPoints, "PointNormal_PointSurfel");
    defineFiltersShadowPoints<pcl::PointNormal, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointNormal_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointNormal, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointNormal_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointNormal, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointNormal_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointSurfel, pcl::Normal>(sub_module_ShadowPoints, "PointSurfel_Normal");
    defineFiltersShadowPoints<pcl::PointSurfel, pcl::PointNormal>(sub_module_ShadowPoints, "PointSurfel_PointNormal");
    defineFiltersShadowPoints<pcl::PointSurfel, pcl::PointSurfel>(sub_module_ShadowPoints, "PointSurfel_PointSurfel");
    defineFiltersShadowPoints<pcl::PointSurfel, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointSurfel_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointSurfel, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointSurfel_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointSurfel, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointSurfel_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointWithRange, pcl::Normal>(sub_module_ShadowPoints, "PointWithRange_Normal");
    defineFiltersShadowPoints<pcl::PointWithRange, pcl::PointNormal>(sub_module_ShadowPoints, "PointWithRange_PointNormal");
    defineFiltersShadowPoints<pcl::PointWithRange, pcl::PointSurfel>(sub_module_ShadowPoints, "PointWithRange_PointSurfel");
    defineFiltersShadowPoints<pcl::PointWithRange, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointWithRange_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointWithRange, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointWithRange_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointWithRange, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointWithRange_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointWithScale, pcl::Normal>(sub_module_ShadowPoints, "PointWithScale_Normal");
    defineFiltersShadowPoints<pcl::PointWithScale, pcl::PointNormal>(sub_module_ShadowPoints, "PointWithScale_PointNormal");
    defineFiltersShadowPoints<pcl::PointWithScale, pcl::PointSurfel>(sub_module_ShadowPoints, "PointWithScale_PointSurfel");
    defineFiltersShadowPoints<pcl::PointWithScale, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointWithScale_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointWithScale, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointWithScale_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointWithScale, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointWithScale_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointWithViewpoint, pcl::Normal>(sub_module_ShadowPoints, "PointWithViewpoint_Normal");
    defineFiltersShadowPoints<pcl::PointWithViewpoint, pcl::PointNormal>(sub_module_ShadowPoints, "PointWithViewpoint_PointNormal");
    defineFiltersShadowPoints<pcl::PointWithViewpoint, pcl::PointSurfel>(sub_module_ShadowPoints, "PointWithViewpoint_PointSurfel");
    defineFiltersShadowPoints<pcl::PointWithViewpoint, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointWithViewpoint_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointWithViewpoint, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointWithViewpoint_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointWithViewpoint, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointWithViewpoint_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointXYZ, pcl::Normal>(sub_module_ShadowPoints, "PointXYZ_Normal");
    defineFiltersShadowPoints<pcl::PointXYZ, pcl::PointNormal>(sub_module_ShadowPoints, "PointXYZ_PointNormal");
    defineFiltersShadowPoints<pcl::PointXYZ, pcl::PointSurfel>(sub_module_ShadowPoints, "PointXYZ_PointSurfel");
    defineFiltersShadowPoints<pcl::PointXYZ, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointXYZ_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointXYZ, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZ_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZ_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointXYZHSV, pcl::Normal>(sub_module_ShadowPoints, "PointXYZHSV_Normal");
    defineFiltersShadowPoints<pcl::PointXYZHSV, pcl::PointNormal>(sub_module_ShadowPoints, "PointXYZHSV_PointNormal");
    defineFiltersShadowPoints<pcl::PointXYZHSV, pcl::PointSurfel>(sub_module_ShadowPoints, "PointXYZHSV_PointSurfel");
    defineFiltersShadowPoints<pcl::PointXYZHSV, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointXYZHSV_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointXYZHSV, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZHSV_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointXYZHSV, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZHSV_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointXYZI, pcl::Normal>(sub_module_ShadowPoints, "PointXYZI_Normal");
    defineFiltersShadowPoints<pcl::PointXYZI, pcl::PointNormal>(sub_module_ShadowPoints, "PointXYZI_PointNormal");
    defineFiltersShadowPoints<pcl::PointXYZI, pcl::PointSurfel>(sub_module_ShadowPoints, "PointXYZI_PointSurfel");
    defineFiltersShadowPoints<pcl::PointXYZI, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointXYZI_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointXYZI, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZI_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointXYZI, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZI_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointXYZINormal, pcl::Normal>(sub_module_ShadowPoints, "PointXYZINormal_Normal");
    defineFiltersShadowPoints<pcl::PointXYZINormal, pcl::PointNormal>(sub_module_ShadowPoints, "PointXYZINormal_PointNormal");
    defineFiltersShadowPoints<pcl::PointXYZINormal, pcl::PointSurfel>(sub_module_ShadowPoints, "PointXYZINormal_PointSurfel");
    defineFiltersShadowPoints<pcl::PointXYZINormal, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointXYZINormal_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointXYZINormal, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZINormal_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointXYZINormal, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZINormal_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointXYZL, pcl::Normal>(sub_module_ShadowPoints, "PointXYZL_Normal");
    defineFiltersShadowPoints<pcl::PointXYZL, pcl::PointNormal>(sub_module_ShadowPoints, "PointXYZL_PointNormal");
    defineFiltersShadowPoints<pcl::PointXYZL, pcl::PointSurfel>(sub_module_ShadowPoints, "PointXYZL_PointSurfel");
    defineFiltersShadowPoints<pcl::PointXYZL, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointXYZL_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointXYZL, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZL_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointXYZL, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZL_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointXYZLNormal, pcl::Normal>(sub_module_ShadowPoints, "PointXYZLNormal_Normal");
    defineFiltersShadowPoints<pcl::PointXYZLNormal, pcl::PointNormal>(sub_module_ShadowPoints, "PointXYZLNormal_PointNormal");
    defineFiltersShadowPoints<pcl::PointXYZLNormal, pcl::PointSurfel>(sub_module_ShadowPoints, "PointXYZLNormal_PointSurfel");
    defineFiltersShadowPoints<pcl::PointXYZLNormal, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointXYZLNormal_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointXYZLNormal, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZLNormal_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointXYZLNormal, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZLNormal_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGB, pcl::Normal>(sub_module_ShadowPoints, "PointXYZRGB_Normal");
    defineFiltersShadowPoints<pcl::PointXYZRGB, pcl::PointNormal>(sub_module_ShadowPoints, "PointXYZRGB_PointNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGB, pcl::PointSurfel>(sub_module_ShadowPoints, "PointXYZRGB_PointSurfel");
    defineFiltersShadowPoints<pcl::PointXYZRGB, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointXYZRGB_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointXYZRGB, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZRGB_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZRGB_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBA, pcl::Normal>(sub_module_ShadowPoints, "PointXYZRGBA_Normal");
    defineFiltersShadowPoints<pcl::PointXYZRGBA, pcl::PointNormal>(sub_module_ShadowPoints, "PointXYZRGBA_PointNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBA, pcl::PointSurfel>(sub_module_ShadowPoints, "PointXYZRGBA_PointSurfel");
    defineFiltersShadowPoints<pcl::PointXYZRGBA, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointXYZRGBA_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBA, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZRGBA_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZRGBA_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBL, pcl::Normal>(sub_module_ShadowPoints, "PointXYZRGBL_Normal");
    defineFiltersShadowPoints<pcl::PointXYZRGBL, pcl::PointNormal>(sub_module_ShadowPoints, "PointXYZRGBL_PointNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBL, pcl::PointSurfel>(sub_module_ShadowPoints, "PointXYZRGBL_PointSurfel");
    defineFiltersShadowPoints<pcl::PointXYZRGBL, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointXYZRGBL_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBL, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZRGBL_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBL, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZRGBL_PointXYZRGBNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBNormal, pcl::Normal>(sub_module_ShadowPoints, "PointXYZRGBNormal_Normal");
    defineFiltersShadowPoints<pcl::PointXYZRGBNormal, pcl::PointNormal>(sub_module_ShadowPoints, "PointXYZRGBNormal_PointNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBNormal, pcl::PointSurfel>(sub_module_ShadowPoints, "PointXYZRGBNormal_PointSurfel");
    defineFiltersShadowPoints<pcl::PointXYZRGBNormal, pcl::PointXYZINormal>(sub_module_ShadowPoints, "PointXYZRGBNormal_PointXYZINormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBNormal, pcl::PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZRGBNormal_PointXYZLNormal");
    defineFiltersShadowPoints<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZRGBNormal_PointXYZRGBNormal");
}