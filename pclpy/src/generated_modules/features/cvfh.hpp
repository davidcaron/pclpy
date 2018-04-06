
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/cvfh.h>



template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
void defineFeaturesCVFHEstimation(py::module &m, std::string const & suffix) {
    using Class = CVFHEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using KdTreePtr = Class::KdTreePtr;
    using VFHEstimator = Class::VFHEstimator;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("view_point", &Class::getViewPoint, &Class::setViewPoint);
    cls.def("set_radius_normals", &Class::setRadiusNormals);
    cls.def("set_cluster_tolerance", &Class::setClusterTolerance);
    cls.def("set_eps_angle_threshold", &Class::setEPSAngleThreshold);
    cls.def("set_curvature_threshold", &Class::setCurvatureThreshold);
    cls.def("set_min_points", &Class::setMinPoints);
    cls.def("set_normalize_bins", &Class::setNormalizeBins);
    cls.def("filter_normals_with_high_curvature", &Class::filterNormalsWithHighCurvature);
    cls.def("compute", &Class::compute);
        
}

void defineFeaturesCvfhClasses(py::module &sub_module) {
    py::module sub_module_CVFHEstimation = sub_module.def_submodule("CVFHEstimation", "Submodule CVFHEstimation");
    defineFeaturesCVFHEstimation<PointXYZ, Normal, VFHSignature308>(sub_module_CVFHEstimation, "PointXYZ_Normal_VFHSignature308");
    defineFeaturesCVFHEstimation<PointXYZI, Normal, VFHSignature308>(sub_module_CVFHEstimation, "PointXYZI_Normal_VFHSignature308");
    defineFeaturesCVFHEstimation<PointXYZRGBA, Normal, VFHSignature308>(sub_module_CVFHEstimation, "PointXYZRGBA_Normal_VFHSignature308");
}