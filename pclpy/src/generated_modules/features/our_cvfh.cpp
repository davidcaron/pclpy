
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

#include <pcl/features/our_cvfh.h>



template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
void defineFeaturesOURCVFHEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::OURCVFHEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using KdTreePtr = Class::KdTreePtr;
    using PointInTPtr = Class::PointInTPtr;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("createTransFromAxes", &Class::createTransFromAxes, "evx"_a, "evy"_a, "evz"_a, "transformPC"_a, "center_mat"_a);
    cls.def("computeRFAndShapeDistribution", &Class::computeRFAndShapeDistribution, "processed"_a, "output"_a, "cluster_indices"_a);
    cls.def("sgurf", &Class::sgurf, "centroid"_a, "normal_centroid"_a, "processed"_a, "transformations"_a, "grid"_a, "indices"_a);
    cls.def("filterNormalsWithHighCurvature", &Class::filterNormalsWithHighCurvature, "cloud"_a, "indices_to_use"_a, "indices_out"_a, "indices_in"_a, "threshold"_a);
    cls.def("compute", &Class::compute, "output"_a);
    cls.def("setViewPoint", &Class::setViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
    cls.def("setRadiusNormals", &Class::setRadiusNormals, "radius_normals"_a);
    cls.def("setClusterTolerance", &Class::setClusterTolerance, "d"_a);
    cls.def("setEPSAngleThreshold", &Class::setEPSAngleThreshold, "d"_a);
    cls.def("setCurvatureThreshold", &Class::setCurvatureThreshold, "d"_a);
    cls.def("setMinPoints", &Class::setMinPoints, "min"_a);
    cls.def("setNormalizeBins", &Class::setNormalizeBins, "normalize"_a);
    cls.def("setRefineClusters", &Class::setRefineClusters, "rc"_a);
    cls.def("setAxisRatio", &Class::setAxisRatio, "f"_a);
    cls.def("setMinAxisValue", &Class::setMinAxisValue, "f"_a);
    cls.def("getViewPoint", &Class::getViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
    cls.def("getCentroidClusters", &Class::getCentroidClusters, "centroids"_a);
    cls.def("getCentroidNormalClusters", &Class::getCentroidNormalClusters, "centroids"_a);
    cls.def("getClusterIndices", &Class::getClusterIndices, "indices"_a);
    cls.def("getClusterAxes", &Class::getClusterAxes, "cluster_axes"_a);
    cls.def("getTransforms", &Class::getTransforms, "trans"_a);
    cls.def("getValidTransformsVec", &Class::getValidTransformsVec, "valid"_a);
        
}

void defineFeaturesOurCvfhFunctions(py::module &m) {
}

void defineFeaturesOurCvfhClasses(py::module &sub_module) {
    py::module sub_module_OURCVFHEstimation = sub_module.def_submodule("OURCVFHEstimation", "Submodule OURCVFHEstimation");
    defineFeaturesOURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308>(sub_module_OURCVFHEstimation, "PointXYZ_Normal_VFHSignature308");
    defineFeaturesOURCVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308>(sub_module_OURCVFHEstimation, "PointXYZRGB_Normal_VFHSignature308");
}