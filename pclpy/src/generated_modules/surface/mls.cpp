
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/surface/mls.h>



template <typename PointInT, typename PointOutT>
void defineSurfaceMovingLeastSquares(py::module &m, std::string const & suffix) {
    using Class = pcl::MovingLeastSquares<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using NormalCloud = Class::NormalCloud;
    using NormalCloudPtr = Class::NormalCloudPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudOutPtr = Class::PointCloudOutPtr;
    using PointCloudOutConstPtr = Class::PointCloudOutConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, pcl::CloudSurfaceProcessing<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::UpsamplingMethod>(cls, "UpsamplingMethod")
        .value("NONE", Class::UpsamplingMethod::NONE)
        .value("DISTINCT_CLOUD", Class::UpsamplingMethod::DISTINCT_CLOUD)
        .value("SAMPLE_LOCAL_PLANE", Class::UpsamplingMethod::SAMPLE_LOCAL_PLANE)
        .value("RANDOM_UNIFORM_DENSITY", Class::UpsamplingMethod::RANDOM_UNIFORM_DENSITY)
        .value("VOXEL_GRID_DILATION", Class::UpsamplingMethod::VOXEL_GRID_DILATION)
        .export_values();
    cls.def(py::init<>());
    cls.def("process", &Class::process, "output"_a);
    cls.def("setComputeNormals", &Class::setComputeNormals, "compute_normals"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("setPolynomialOrder", &Class::setPolynomialOrder, "order"_a);
    cls.def("setPolynomialFit", &Class::setPolynomialFit, "polynomial_fit"_a);
    cls.def("setSearchRadius", &Class::setSearchRadius, "radius"_a);
    cls.def("setSqrGaussParam", &Class::setSqrGaussParam, "sqr_gauss_param"_a);
    cls.def("setUpsamplingMethod", &Class::setUpsamplingMethod, "method"_a);
    cls.def("setDistinctCloud", &Class::setDistinctCloud, "distinct_cloud"_a);
    cls.def("setUpsamplingRadius", &Class::setUpsamplingRadius, "radius"_a);
    cls.def("setUpsamplingStepSize", &Class::setUpsamplingStepSize, "step_size"_a);
    cls.def("setPointDensity", &Class::setPointDensity, "desired_num_points_in_radius"_a);
    cls.def("setDilationVoxelSize", &Class::setDilationVoxelSize, "voxel_size"_a);
    cls.def("setDilationIterations", &Class::setDilationIterations, "iterations"_a);
    cls.def("getSearchMethod", &Class::getSearchMethod);
    cls.def("getPolynomialOrder", &Class::getPolynomialOrder);
    cls.def("getPolynomialFit", &Class::getPolynomialFit);
    cls.def("getSearchRadius", &Class::getSearchRadius);
    cls.def("getSqrGaussParam", &Class::getSqrGaussParam);
    cls.def("getDistinctCloud", &Class::getDistinctCloud);
    cls.def("getUpsamplingRadius", &Class::getUpsamplingRadius);
    cls.def("getUpsamplingStepSize", &Class::getUpsamplingStepSize);
    cls.def("getPointDensity", &Class::getPointDensity);
    cls.def("getDilationVoxelSize", &Class::getDilationVoxelSize);
    cls.def("getDilationIterations", &Class::getDilationIterations);
    cls.def("getCorrespondingIndices", &Class::getCorrespondingIndices);
        
}

template <typename PointInT, typename PointOutT>
void defineSurfaceMovingLeastSquaresOMP(py::module &m, std::string const & suffix) {
    using Class = pcl::MovingLeastSquaresOMP<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using NormalCloud = Class::NormalCloud;
    using NormalCloudPtr = Class::NormalCloudPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudOutPtr = Class::PointCloudOutPtr;
    using PointCloudOutConstPtr = Class::PointCloudOutConstPtr;
    py::class_<Class, pcl::MovingLeastSquares<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "threads"_a=0);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "threads"_a=0);
        
}

void defineSurfaceMlsFunctions(py::module &m) {
}

void defineSurfaceMlsClasses(py::module &sub_module) {
    py::module sub_module_MovingLeastSquares = sub_module.def_submodule("MovingLeastSquares", "Submodule MovingLeastSquares");
    defineSurfaceMovingLeastSquares<pcl::PointNormal, pcl::PointNormal>(sub_module_MovingLeastSquares, "PointNormal_PointNormal");
    defineSurfaceMovingLeastSquares<pcl::PointNormal, pcl::PointXYZ>(sub_module_MovingLeastSquares, "PointNormal_PointXYZ");
    defineSurfaceMovingLeastSquares<pcl::PointNormal, pcl::PointXYZRGB>(sub_module_MovingLeastSquares, "PointNormal_PointXYZRGB");
    defineSurfaceMovingLeastSquares<pcl::PointNormal, pcl::PointXYZRGBA>(sub_module_MovingLeastSquares, "PointNormal_PointXYZRGBA");
    defineSurfaceMovingLeastSquares<pcl::PointNormal, pcl::PointXYZRGBNormal>(sub_module_MovingLeastSquares, "PointNormal_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>(sub_module_MovingLeastSquares, "PointXYZ_PointNormal");
    defineSurfaceMovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>(sub_module_MovingLeastSquares, "PointXYZ_PointXYZ");
    defineSurfaceMovingLeastSquares<pcl::PointXYZ, pcl::PointXYZRGB>(sub_module_MovingLeastSquares, "PointXYZ_PointXYZRGB");
    defineSurfaceMovingLeastSquares<pcl::PointXYZ, pcl::PointXYZRGBA>(sub_module_MovingLeastSquares, "PointXYZ_PointXYZRGBA");
    defineSurfaceMovingLeastSquares<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_MovingLeastSquares, "PointXYZ_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal>(sub_module_MovingLeastSquares, "PointXYZRGB_PointNormal");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZ>(sub_module_MovingLeastSquares, "PointXYZRGB_PointXYZ");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>(sub_module_MovingLeastSquares, "PointXYZRGB_PointXYZRGB");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBA>(sub_module_MovingLeastSquares, "PointXYZRGB_PointXYZRGBA");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_MovingLeastSquares, "PointXYZRGB_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGBA, pcl::PointNormal>(sub_module_MovingLeastSquares, "PointXYZRGBA_PointNormal");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZ>(sub_module_MovingLeastSquares, "PointXYZRGBA_PointXYZ");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGB>(sub_module_MovingLeastSquares, "PointXYZRGBA_PointXYZRGB");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(sub_module_MovingLeastSquares, "PointXYZRGBA_PointXYZRGBA");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_MovingLeastSquares, "PointXYZRGBA_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointNormal>(sub_module_MovingLeastSquares, "PointXYZRGBNormal_PointNormal");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZ>(sub_module_MovingLeastSquares, "PointXYZRGBNormal_PointXYZ");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(sub_module_MovingLeastSquares, "PointXYZRGBNormal_PointXYZRGB");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZRGBA>(sub_module_MovingLeastSquares, "PointXYZRGBNormal_PointXYZRGBA");
    defineSurfaceMovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(sub_module_MovingLeastSquares, "PointXYZRGBNormal_PointXYZRGBNormal");
    py::module sub_module_MovingLeastSquaresOMP = sub_module.def_submodule("MovingLeastSquaresOMP", "Submodule MovingLeastSquaresOMP");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointNormal, pcl::PointNormal>(sub_module_MovingLeastSquaresOMP, "PointNormal_PointNormal");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointNormal, pcl::PointXYZ>(sub_module_MovingLeastSquaresOMP, "PointNormal_PointXYZ");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointNormal, pcl::PointXYZRGB>(sub_module_MovingLeastSquaresOMP, "PointNormal_PointXYZRGB");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointNormal, pcl::PointXYZRGBA>(sub_module_MovingLeastSquaresOMP, "PointNormal_PointXYZRGBA");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointNormal, pcl::PointXYZRGBNormal>(sub_module_MovingLeastSquaresOMP, "PointNormal_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZ_PointNormal");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointXYZ>(sub_module_MovingLeastSquaresOMP, "PointXYZ_PointXYZ");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointXYZRGB>(sub_module_MovingLeastSquaresOMP, "PointXYZ_PointXYZRGB");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointXYZRGBA>(sub_module_MovingLeastSquaresOMP, "PointXYZ_PointXYZRGBA");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZ_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGB, pcl::PointNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGB_PointNormal");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGB, pcl::PointXYZ>(sub_module_MovingLeastSquaresOMP, "PointXYZRGB_PointXYZ");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGB, pcl::PointXYZRGB>(sub_module_MovingLeastSquaresOMP, "PointXYZRGB_PointXYZRGB");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGB, pcl::PointXYZRGBA>(sub_module_MovingLeastSquaresOMP, "PointXYZRGB_PointXYZRGBA");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGB_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGBA, pcl::PointNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBA_PointNormal");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGBA, pcl::PointXYZ>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBA_PointXYZ");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGBA, pcl::PointXYZRGB>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBA_PointXYZRGB");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBA_PointXYZRGBA");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBA_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGBNormal, pcl::PointNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBNormal_PointNormal");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGBNormal, pcl::PointXYZ>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBNormal_PointXYZ");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBNormal_PointXYZRGB");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBA>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBNormal_PointXYZRGBA");
    defineSurfaceMovingLeastSquaresOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBNormal_PointXYZRGBNormal");
}