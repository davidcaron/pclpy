
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

#include <pcl/filters/normal_space.h>



template<typename PointT, typename NormalT>
void defineFiltersNormalSpaceSampling(py::module &m, std::string const & suffix) {
    using Class = pcl::NormalSpaceSampling<PointT, NormalT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setSample", &Class::setSample, "sample"_a);
    cls.def("setSeed", &Class::setSeed, "seed"_a);
    cls.def("setBins", &Class::setBins, "binsx"_a, "binsy"_a, "binsz"_a);
    cls.def("setNormals", &Class::setNormals, "normals"_a);
    cls.def("getSample", &Class::getSample);
    cls.def("getSeed", &Class::getSeed);
    cls.def("getBins", &Class::getBins, "binsx"_a, "binsy"_a, "binsz"_a);
    cls.def("getNormals", &Class::getNormals);
        
}

void defineFiltersNormalSpaceFunctions(py::module &m) {
}

void defineFiltersNormalSpaceClasses(py::module &sub_module) {
    py::module sub_module_NormalSpaceSampling = sub_module.def_submodule("NormalSpaceSampling", "Submodule NormalSpaceSampling");
    defineFiltersNormalSpaceSampling<pcl::InterestPoint, pcl::Normal>(sub_module_NormalSpaceSampling, "InterestPoint_Normal");
    defineFiltersNormalSpaceSampling<pcl::InterestPoint, pcl::PointNormal>(sub_module_NormalSpaceSampling, "InterestPoint_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::InterestPoint, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "InterestPoint_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::InterestPoint, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "InterestPoint_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::InterestPoint, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "InterestPoint_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::InterestPoint, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "InterestPoint_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointDEM, pcl::Normal>(sub_module_NormalSpaceSampling, "PointDEM_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointDEM, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointDEM_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointDEM, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointDEM_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointDEM, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointDEM_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointDEM, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointDEM_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointDEM, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointDEM_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointNormal, pcl::Normal>(sub_module_NormalSpaceSampling, "PointNormal_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointNormal, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointNormal_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointNormal, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointNormal_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointNormal, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointNormal_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointNormal, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointNormal_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointNormal, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointNormal_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointSurfel, pcl::Normal>(sub_module_NormalSpaceSampling, "PointSurfel_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointSurfel, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointSurfel_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointSurfel, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointSurfel_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointSurfel, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointSurfel_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointSurfel, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointSurfel_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointSurfel, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointSurfel_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithRange, pcl::Normal>(sub_module_NormalSpaceSampling, "PointWithRange_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointWithRange, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointWithRange_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithRange, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointWithRange_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointWithRange, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointWithRange_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithRange, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointWithRange_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithRange, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointWithRange_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithScale, pcl::Normal>(sub_module_NormalSpaceSampling, "PointWithScale_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointWithScale, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointWithScale_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithScale, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointWithScale_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointWithScale, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointWithScale_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithScale, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointWithScale_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithScale, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointWithScale_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithViewpoint, pcl::Normal>(sub_module_NormalSpaceSampling, "PointWithViewpoint_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointWithViewpoint, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointWithViewpoint_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithViewpoint, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointWithViewpoint_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointWithViewpoint, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointWithViewpoint_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithViewpoint, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointWithViewpoint_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointWithViewpoint, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointWithViewpoint_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZ, pcl::Normal>(sub_module_NormalSpaceSampling, "PointXYZ_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZ, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointXYZ_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZ, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZ_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointXYZ, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZ_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZ, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZ_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZ_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZHSV, pcl::Normal>(sub_module_NormalSpaceSampling, "PointXYZHSV_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZHSV, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointXYZHSV_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZHSV, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZHSV_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointXYZHSV, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZHSV_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZHSV, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZHSV_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZHSV, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZHSV_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZI, pcl::Normal>(sub_module_NormalSpaceSampling, "PointXYZI_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZI, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointXYZI_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZI, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZI_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointXYZI, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZI_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZI, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZI_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZI, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZI_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZINormal, pcl::Normal>(sub_module_NormalSpaceSampling, "PointXYZINormal_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZINormal, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointXYZINormal_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZINormal, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZINormal_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointXYZINormal, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZINormal_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZINormal, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZINormal_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZINormal, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZINormal_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZL, pcl::Normal>(sub_module_NormalSpaceSampling, "PointXYZL_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZL, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointXYZL_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZL, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZL_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointXYZL, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZL_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZL, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZL_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZL, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZL_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZLNormal, pcl::Normal>(sub_module_NormalSpaceSampling, "PointXYZLNormal_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZLNormal, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointXYZLNormal_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZLNormal, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZLNormal_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointXYZLNormal, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZLNormal_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZLNormal, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZLNormal_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZLNormal, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZLNormal_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGB, pcl::Normal>(sub_module_NormalSpaceSampling, "PointXYZRGB_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGB, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointXYZRGB_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGB, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZRGB_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGB, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZRGB_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGB, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZRGB_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZRGB_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBA, pcl::Normal>(sub_module_NormalSpaceSampling, "PointXYZRGBA_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBA, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBA_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBA, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZRGBA_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBA, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZRGBA_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBA, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBA_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBA_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBL, pcl::Normal>(sub_module_NormalSpaceSampling, "PointXYZRGBL_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBL, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBL_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBL, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZRGBL_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBL, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZRGBL_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBL, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBL_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBL, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBL_PointXYZRGBNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBNormal, pcl::Normal>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_Normal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBNormal, pcl::PointNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_PointNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBNormal, pcl::PointSurfel>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_PointSurfel");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBNormal, pcl::PointXYZINormal>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_PointXYZINormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBNormal, pcl::PointXYZLNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_PointXYZLNormal");
    defineFiltersNormalSpaceSampling<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(sub_module_NormalSpaceSampling, "PointXYZRGBNormal_PointXYZRGBNormal");
}