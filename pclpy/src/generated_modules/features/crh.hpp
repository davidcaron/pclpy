
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/crh.h>



template<typename PointInT, typename PointNT, typename PointOutT = pcl::Histogram<90> >
void defineFeaturesCRHEstimation(py::module &m, std::string const & suffix) {
    using Class = CRHEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("view_point", &Class::getViewPoint, &Class::setViewPoint);
    cls.def("set_centroid", &Class::setCentroid);
        
}

void defineFeaturesCrhClasses(py::module &sub_module) {
    py::module sub_module_CRHEstimation = sub_module.def_submodule("CRHEstimation", "Submodule CRHEstimation");
    defineFeaturesCRHEstimation<PointXYZ, Normal, Histogram<90>>(sub_module_CRHEstimation, "PointXYZ_Normal_Histogram<90>");
    defineFeaturesCRHEstimation<PointXYZI, Normal, Histogram<90>>(sub_module_CRHEstimation, "PointXYZI_Normal_Histogram<90>");
    defineFeaturesCRHEstimation<PointXYZRGBA, Normal, Histogram<90>>(sub_module_CRHEstimation, "PointXYZRGBA_Normal_Histogram<90>");
}