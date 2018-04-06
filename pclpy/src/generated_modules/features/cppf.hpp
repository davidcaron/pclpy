
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/cppf.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesCPPFEstimation(py::module &m, std::string const & suffix) {
    using Class = CPPFEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineFeaturesCppfClasses(py::module &sub_module) {
    py::module sub_module_CPPFEstimation = sub_module.def_submodule("CPPFEstimation", "Submodule CPPFEstimation");
    defineFeaturesCPPFEstimation<PointXYZRGBA, Normal, CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBA_Normal_CPPFSignature");
    defineFeaturesCPPFEstimation<PointXYZRGBA, PointNormal, CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBA_PointNormal_CPPFSignature");
    defineFeaturesCPPFEstimation<PointXYZRGBA, PointXYZRGBNormal, CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBA_PointXYZRGBNormal_CPPFSignature");
    defineFeaturesCPPFEstimation<PointXYZRGBNormal, Normal, CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBNormal_Normal_CPPFSignature");
    defineFeaturesCPPFEstimation<PointXYZRGBNormal, PointNormal, CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBNormal_PointNormal_CPPFSignature");
    defineFeaturesCPPFEstimation<PointXYZRGBNormal, PointXYZRGBNormal, CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBNormal_PointXYZRGBNormal_CPPFSignature");
}