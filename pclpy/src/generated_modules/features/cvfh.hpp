
#include <pcl/features/cvfh.h>



template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
void defineFeaturesCVFHEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::CVFHEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using KdTreePtr = Class::KdTreePtr;
    using VFHEstimator = Class::VFHEstimator;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("filterNormalsWithHighCurvature", &Class::filterNormalsWithHighCurvature, "cloud"_a, "indices_to_use"_a, "indices_out"_a, "indices_in"_a, "threshold"_a);
    cls.def("compute", &Class::compute, "output"_a);
    cls.def("setViewPoint", &Class::setViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
    cls.def("setRadiusNormals", &Class::setRadiusNormals, "radius_normals"_a);
    cls.def("setClusterTolerance", &Class::setClusterTolerance, "d"_a);
    cls.def("setEPSAngleThreshold", &Class::setEPSAngleThreshold, "d"_a);
    cls.def("setCurvatureThreshold", &Class::setCurvatureThreshold, "d"_a);
    cls.def("setMinPoints", &Class::setMinPoints, "min"_a);
    cls.def("setNormalizeBins", &Class::setNormalizeBins, "normalize"_a);
    cls.def("getViewPoint", &Class::getViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
    cls.def("getCentroidClusters", &Class::getCentroidClusters, "centroids"_a);
    cls.def("getCentroidNormalClusters", &Class::getCentroidNormalClusters, "centroids"_a);
        
}

void defineFeaturesCvfhFunctions(py::module &m) {
}

void defineFeaturesCvfhClasses(py::module &sub_module) {
    py::module sub_module_CVFHEstimation = sub_module.def_submodule("CVFHEstimation", "Submodule CVFHEstimation");
    defineFeaturesCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308>(sub_module_CVFHEstimation, "PointXYZ_Normal_VFHSignature308");
    defineFeaturesCVFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::VFHSignature308>(sub_module_CVFHEstimation, "PointXYZI_Normal_VFHSignature308");
    defineFeaturesCVFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::VFHSignature308>(sub_module_CVFHEstimation, "PointXYZRGBA_Normal_VFHSignature308");
}