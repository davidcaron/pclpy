
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

#include <pcl/features/crh.h>



template<typename PointInT, typename PointNT, typename PointOutT = pcl::Histogram<90> >
void defineFeaturesCRHEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::CRHEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setViewPoint", &Class::setViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
    cls.def("setCentroid", &Class::setCentroid, "centroid"_a);
    cls.def("getViewPoint", &Class::getViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
        
}

void defineFeaturesCrhFunctions(py::module &m) {
}

void defineFeaturesCrhClasses(py::module &sub_module) {
    py::module sub_module_CRHEstimation = sub_module.def_submodule("CRHEstimation", "Submodule CRHEstimation");
    defineFeaturesCRHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<90>>(sub_module_CRHEstimation, "PointXYZ_Normal_Histogram<90>");
    defineFeaturesCRHEstimation<pcl::PointXYZI, pcl::Normal, pcl::Histogram<90>>(sub_module_CRHEstimation, "PointXYZI_Normal_Histogram<90>");
    defineFeaturesCRHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Histogram<90>>(sub_module_CRHEstimation, "PointXYZRGBA_Normal_Histogram<90>");
}