
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/pfhrgb.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::PFHRGBSignature250>
void defineFeaturesPFHRGBEstimation(py::module &m, std::string const & suffix) {
    using Class = PFHRGBEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute_rgb_pair_features", &Class::computeRGBPairFeatures);
    cls.def("compute_point_pfhrgb_signature", &Class::computePointPFHRGBSignature);
        
}

void defineFeaturesPfhrgbClasses(py::module &sub_module) {
    py::module sub_module_PFHRGBEstimation = sub_module.def_submodule("PFHRGBEstimation", "Submodule PFHRGBEstimation");
    defineFeaturesPFHRGBEstimation<PointXYZRGB, Normal, PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGB_Normal_PFHRGBSignature250");
    defineFeaturesPFHRGBEstimation<PointXYZRGB, PointXYZRGBNormal, PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGB_PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesPFHRGBEstimation<PointXYZRGBA, Normal, PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGBA_Normal_PFHRGBSignature250");
    defineFeaturesPFHRGBEstimation<PointXYZRGBA, PointXYZRGBNormal, PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGBA_PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesPFHRGBEstimation<PointXYZRGBNormal, Normal, PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGBNormal_Normal_PFHRGBSignature250");
    defineFeaturesPFHRGBEstimation<PointXYZRGBNormal, PointXYZRGBNormal, PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGBNormal_PointXYZRGBNormal_PFHRGBSignature250");
}