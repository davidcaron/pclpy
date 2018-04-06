
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/normal_space.h>



template<typename PointT, typename NormalT>
void defineFiltersNormalSpaceSampling(py::module &m, std::string const & suffix) {
    using Class = NormalSpaceSampling<PointT, NormalT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("sample", &Class::getSample, &Class::setSample);
    cls.def_property("seed", &Class::getSeed, &Class::setSeed);
    cls.def_property("bins", &Class::getBins, &Class::setBins);
    cls.def_property("normals", &Class::getNormals, &Class::setNormals);
        
}

void defineFiltersNormalSpaceClasses(py::module &sub_module) {
    py::module sub_module_NormalSpaceSampling = sub_module.def_submodule("NormalSpaceSampling", "Submodule NormalSpaceSampling");
    defineFiltersNormalSpaceSampling<InterestPoint, Normal>(sub_module_NormalSpaceSampling, "InterestPoint_Normal");
    defineFiltersNormalSpaceSampling<InterestPoint, PointNormal>(sub_module_NormalSpaceSampling, "InterestPoint_PointNormal");
    defineFiltersNormalSpaceSampling<InterestPoint, PointSurfel>(sub_module_NormalSpaceSampling, "InterestPoint_PointSurfel");
    defineFiltersNormalSpaceSampling<InterestPoint, PointXYZINormal>(sub_module_NormalSpaceSampling, "InterestPoint_PointXYZINormal");
    defineFiltersNormalSpaceSampling<InterestPoint, PointXYZLNormal>(sub_module_NormalSpaceSampling, "InterestPoint_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<InterestPoint, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "InterestPoint_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointDEM, Normal>(sub_module_NormalSpaceSampling, "PointDEM_Normal");
    defineFiltersNormalSpaceSampling<PointDEM, PointNormal>(sub_module_NormalSpaceSampling, "PointDEM_PointNormal");
    defineFiltersNormalSpaceSampling<PointDEM, PointSurfel>(sub_module_NormalSpaceSampling, "PointDEM_PointSurfel");
    defineFiltersNormalSpaceSampling<PointDEM, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointDEM_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointDEM, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointDEM_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointDEM, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointDEM_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointNormal, Normal>(sub_module_NormalSpaceSampling, "PointNormal_Normal");
    defineFiltersNormalSpaceSampling<PointNormal, PointNormal>(sub_module_NormalSpaceSampling, "PointNormal_PointNormal");
    defineFiltersNormalSpaceSampling<PointNormal, PointSurfel>(sub_module_NormalSpaceSampling, "PointNormal_PointSurfel");
    defineFiltersNormalSpaceSampling<PointNormal, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointNormal_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointNormal, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointNormal_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointNormal, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointNormal_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointSurfel, Normal>(sub_module_NormalSpaceSampling, "PointSurfel_Normal");
    defineFiltersNormalSpaceSampling<PointSurfel, PointNormal>(sub_module_NormalSpaceSampling, "PointSurfel_PointNormal");
    defineFiltersNormalSpaceSampling<PointSurfel, PointSurfel>(sub_module_NormalSpaceSampling, "PointSurfel_PointSurfel");
    defineFiltersNormalSpaceSampling<PointSurfel, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointSurfel_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointSurfel, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointSurfel_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointSurfel, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointSurfel_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointWithRange, Normal>(sub_module_NormalSpaceSampling, "PointWithRange_Normal");
    defineFiltersNormalSpaceSampling<PointWithRange, PointNormal>(sub_module_NormalSpaceSampling, "PointWithRange_PointNormal");
    defineFiltersNormalSpaceSampling<PointWithRange, PointSurfel>(sub_module_NormalSpaceSampling, "PointWithRange_PointSurfel");
    defineFiltersNormalSpaceSampling<PointWithRange, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointWithRange_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointWithRange, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointWithRange_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointWithRange, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointWithRange_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointWithScale, Normal>(sub_module_NormalSpaceSampling, "PointWithScale_Normal");
    defineFiltersNormalSpaceSampling<PointWithScale, PointNormal>(sub_module_NormalSpaceSampling, "PointWithScale_PointNormal");
    defineFiltersNormalSpaceSampling<PointWithScale, PointSurfel>(sub_module_NormalSpaceSampling, "PointWithScale_PointSurfel");
    defineFiltersNormalSpaceSampling<PointWithScale, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointWithScale_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointWithScale, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointWithScale_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointWithScale, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointWithScale_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointWithViewpoint, Normal>(sub_module_NormalSpaceSampling, "PointWithViewpoint_Normal");
    defineFiltersNormalSpaceSampling<PointWithViewpoint, PointNormal>(sub_module_NormalSpaceSampling, "PointWithViewpoint_PointNormal");
    defineFiltersNormalSpaceSampling<PointWithViewpoint, PointSurfel>(sub_module_NormalSpaceSampling, "PointWithViewpoint_PointSurfel");
    defineFiltersNormalSpaceSampling<PointWithViewpoint, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointWithViewpoint_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointWithViewpoint, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointWithViewpoint_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointWithViewpoint, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointWithViewpoint_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointXYZ, Normal>(sub_module_NormalSpaceSampling, "PointXYZ_Normal");
    defineFiltersNormalSpaceSampling<PointXYZ, PointNormal>(sub_module_NormalSpaceSampling, "PointXYZ_PointNormal");
    defineFiltersNormalSpaceSampling<PointXYZ, PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZ_PointSurfel");
    defineFiltersNormalSpaceSampling<PointXYZ, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZ_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointXYZ, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZ_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointXYZ, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZ_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointXYZHSV, Normal>(sub_module_NormalSpaceSampling, "PointXYZHSV_Normal");
    defineFiltersNormalSpaceSampling<PointXYZHSV, PointNormal>(sub_module_NormalSpaceSampling, "PointXYZHSV_PointNormal");
    defineFiltersNormalSpaceSampling<PointXYZHSV, PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZHSV_PointSurfel");
    defineFiltersNormalSpaceSampling<PointXYZHSV, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZHSV_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointXYZHSV, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZHSV_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointXYZHSV, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZHSV_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointXYZI, Normal>(sub_module_NormalSpaceSampling, "PointXYZI_Normal");
    defineFiltersNormalSpaceSampling<PointXYZI, PointNormal>(sub_module_NormalSpaceSampling, "PointXYZI_PointNormal");
    defineFiltersNormalSpaceSampling<PointXYZI, PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZI_PointSurfel");
    defineFiltersNormalSpaceSampling<PointXYZI, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZI_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointXYZI, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZI_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointXYZI, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZI_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointXYZINormal, Normal>(sub_module_NormalSpaceSampling, "PointXYZINormal_Normal");
    defineFiltersNormalSpaceSampling<PointXYZINormal, PointNormal>(sub_module_NormalSpaceSampling, "PointXYZINormal_PointNormal");
    defineFiltersNormalSpaceSampling<PointXYZINormal, PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZINormal_PointSurfel");
    defineFiltersNormalSpaceSampling<PointXYZINormal, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZINormal_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointXYZINormal, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZINormal_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointXYZINormal, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZINormal_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointXYZL, Normal>(sub_module_NormalSpaceSampling, "PointXYZL_Normal");
    defineFiltersNormalSpaceSampling<PointXYZL, PointNormal>(sub_module_NormalSpaceSampling, "PointXYZL_PointNormal");
    defineFiltersNormalSpaceSampling<PointXYZL, PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZL_PointSurfel");
    defineFiltersNormalSpaceSampling<PointXYZL, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZL_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointXYZL, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZL_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointXYZL, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZL_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointXYZLNormal, Normal>(sub_module_NormalSpaceSampling, "PointXYZLNormal_Normal");
    defineFiltersNormalSpaceSampling<PointXYZLNormal, PointNormal>(sub_module_NormalSpaceSampling, "PointXYZLNormal_PointNormal");
    defineFiltersNormalSpaceSampling<PointXYZLNormal, PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZLNormal_PointSurfel");
    defineFiltersNormalSpaceSampling<PointXYZLNormal, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZLNormal_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointXYZLNormal, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZLNormal_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointXYZLNormal, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZLNormal_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGB, Normal>(sub_module_NormalSpaceSampling, "PointXYZRGB_Normal");
    defineFiltersNormalSpaceSampling<PointXYZRGB, PointNormal>(sub_module_NormalSpaceSampling, "PointXYZRGB_PointNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGB, PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZRGB_PointSurfel");
    defineFiltersNormalSpaceSampling<PointXYZRGB, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZRGB_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointXYZRGB, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZRGB_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGB, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZRGB_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBA, Normal>(sub_module_NormalSpaceSampling, "PointXYZRGBA_Normal");
    defineFiltersNormalSpaceSampling<PointXYZRGBA, PointNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBA_PointNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBA, PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZRGBA_PointSurfel");
    defineFiltersNormalSpaceSampling<PointXYZRGBA, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZRGBA_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBA, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBA_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBA, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBA_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBL, Normal>(sub_module_NormalSpaceSampling, "PointXYZRGBL_Normal");
    defineFiltersNormalSpaceSampling<PointXYZRGBL, PointNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBL_PointNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBL, PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZRGBL_PointSurfel");
    defineFiltersNormalSpaceSampling<PointXYZRGBL, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZRGBL_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBL, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBL_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBL, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBL_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBNormal, Normal>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_Normal");
    defineFiltersNormalSpaceSampling<PointXYZRGBNormal, PointNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_PointNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBNormal, PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_PointSurfel");
    defineFiltersNormalSpaceSampling<PointXYZRGBNormal, PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_PointXYZINormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBNormal, PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<PointXYZRGBNormal, PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_PointXYZRGBNormal");
}