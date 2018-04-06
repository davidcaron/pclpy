
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/mls.h>



template <typename PointInT, typename PointOutT>
void defineSurfaceMovingLeastSquares(py::module &m, std::string const & suffix) {
    using Class = MovingLeastSquares<PointInT, PointOutT>;
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
    py::class_<Class, CloudSurfaceProcessing<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::UpsamplingMethod>(cls, "upsampling_method")
        .value("NONE", Class::UpsamplingMethod::NONE)
        .value("DISTINCT_CLOUD", Class::UpsamplingMethod::DISTINCT_CLOUD)
        .value("SAMPLE_LOCAL_PLANE", Class::UpsamplingMethod::SAMPLE_LOCAL_PLANE)
        .value("RANDOM_UNIFORM_DENSITY", Class::UpsamplingMethod::RANDOM_UNIFORM_DENSITY)
        .value("VOXEL_GRID_DILATION", Class::UpsamplingMethod::VOXEL_GRID_DILATION)
        .export_values();
    cls.def(py::init<>());
    cls.def("set_compute_normals", &Class::setComputeNormals);
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def_property("polynomial_order", &Class::getPolynomialOrder, &Class::setPolynomialOrder);
    cls.def_property("polynomial_fit", &Class::getPolynomialFit, &Class::setPolynomialFit);
    cls.def_property("search_radius", &Class::getSearchRadius, &Class::setSearchRadius);
    cls.def_property("sqr_gauss_param", &Class::getSqrGaussParam, &Class::setSqrGaussParam);
    cls.def("set_upsampling_method", &Class::setUpsamplingMethod);
    cls.def_property("distinct_cloud", &Class::getDistinctCloud, &Class::setDistinctCloud);
    cls.def_property("upsampling_radius", &Class::getUpsamplingRadius, &Class::setUpsamplingRadius);
    cls.def_property("upsampling_step_size", &Class::getUpsamplingStepSize, &Class::setUpsamplingStepSize);
    cls.def_property("point_density", &Class::getPointDensity, &Class::setPointDensity);
    cls.def_property("dilation_voxel_size", &Class::getDilationVoxelSize, &Class::setDilationVoxelSize);
    cls.def_property("dilation_iterations", &Class::getDilationIterations, &Class::setDilationIterations);
    cls.def("process", &Class::process);
        
}

template <typename PointInT, typename PointOutT>
void defineSurfaceMovingLeastSquaresOMP(py::module &m, std::string const & suffix) {
    using Class = MovingLeastSquaresOMP<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using NormalCloud = Class::NormalCloud;
    using NormalCloudPtr = Class::NormalCloudPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudOutPtr = Class::PointCloudOutPtr;
    using PointCloudOutConstPtr = Class::PointCloudOutConstPtr;
    py::class_<Class, MovingLeastSquares<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "threads"_a=0);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineSurfaceMlsClasses(py::module &sub_module) {
    py::module sub_module_MovingLeastSquares = sub_module.def_submodule("MovingLeastSquares", "Submodule MovingLeastSquares");
    defineSurfaceMovingLeastSquares<PointNormal, PointNormal>(sub_module_MovingLeastSquares, "PointNormal_PointNormal");
    defineSurfaceMovingLeastSquares<PointNormal, PointXYZ>(sub_module_MovingLeastSquares, "PointNormal_PointXYZ");
    defineSurfaceMovingLeastSquares<PointNormal, PointXYZRGB>(sub_module_MovingLeastSquares, "PointNormal_PointXYZRGB");
    defineSurfaceMovingLeastSquares<PointNormal, PointXYZRGBA>(sub_module_MovingLeastSquares, "PointNormal_PointXYZRGBA");
    defineSurfaceMovingLeastSquares<PointNormal, PointXYZRGBNormal>(sub_module_MovingLeastSquares, "PointNormal_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquares<PointXYZ, PointNormal>(sub_module_MovingLeastSquares, "PointXYZ_PointNormal");
    defineSurfaceMovingLeastSquares<PointXYZ, PointXYZ>(sub_module_MovingLeastSquares, "PointXYZ_PointXYZ");
    defineSurfaceMovingLeastSquares<PointXYZ, PointXYZRGB>(sub_module_MovingLeastSquares, "PointXYZ_PointXYZRGB");
    defineSurfaceMovingLeastSquares<PointXYZ, PointXYZRGBA>(sub_module_MovingLeastSquares, "PointXYZ_PointXYZRGBA");
    defineSurfaceMovingLeastSquares<PointXYZ, PointXYZRGBNormal>(sub_module_MovingLeastSquares, "PointXYZ_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquares<PointXYZRGB, PointNormal>(sub_module_MovingLeastSquares, "PointXYZRGB_PointNormal");
    defineSurfaceMovingLeastSquares<PointXYZRGB, PointXYZ>(sub_module_MovingLeastSquares, "PointXYZRGB_PointXYZ");
    defineSurfaceMovingLeastSquares<PointXYZRGB, PointXYZRGB>(sub_module_MovingLeastSquares, "PointXYZRGB_PointXYZRGB");
    defineSurfaceMovingLeastSquares<PointXYZRGB, PointXYZRGBA>(sub_module_MovingLeastSquares, "PointXYZRGB_PointXYZRGBA");
    defineSurfaceMovingLeastSquares<PointXYZRGB, PointXYZRGBNormal>(sub_module_MovingLeastSquares, "PointXYZRGB_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquares<PointXYZRGBA, PointNormal>(sub_module_MovingLeastSquares, "PointXYZRGBA_PointNormal");
    defineSurfaceMovingLeastSquares<PointXYZRGBA, PointXYZ>(sub_module_MovingLeastSquares, "PointXYZRGBA_PointXYZ");
    defineSurfaceMovingLeastSquares<PointXYZRGBA, PointXYZRGB>(sub_module_MovingLeastSquares, "PointXYZRGBA_PointXYZRGB");
    defineSurfaceMovingLeastSquares<PointXYZRGBA, PointXYZRGBA>(sub_module_MovingLeastSquares, "PointXYZRGBA_PointXYZRGBA");
    defineSurfaceMovingLeastSquares<PointXYZRGBA, PointXYZRGBNormal>(sub_module_MovingLeastSquares, "PointXYZRGBA_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquares<PointXYZRGBNormal, PointNormal>(sub_module_MovingLeastSquares, "PointXYZRGBNormal_PointNormal");
    defineSurfaceMovingLeastSquares<PointXYZRGBNormal, PointXYZ>(sub_module_MovingLeastSquares, "PointXYZRGBNormal_PointXYZ");
    defineSurfaceMovingLeastSquares<PointXYZRGBNormal, PointXYZRGB>(sub_module_MovingLeastSquares, "PointXYZRGBNormal_PointXYZRGB");
    defineSurfaceMovingLeastSquares<PointXYZRGBNormal, PointXYZRGBA>(sub_module_MovingLeastSquares, "PointXYZRGBNormal_PointXYZRGBA");
    defineSurfaceMovingLeastSquares<PointXYZRGBNormal, PointXYZRGBNormal>(sub_module_MovingLeastSquares, "PointXYZRGBNormal_PointXYZRGBNormal");
    py::module sub_module_MovingLeastSquaresOMP = sub_module.def_submodule("MovingLeastSquaresOMP", "Submodule MovingLeastSquaresOMP");
    defineSurfaceMovingLeastSquaresOMP<PointNormal, PointNormal>(sub_module_MovingLeastSquaresOMP, "PointNormal_PointNormal");
    defineSurfaceMovingLeastSquaresOMP<PointNormal, PointXYZ>(sub_module_MovingLeastSquaresOMP, "PointNormal_PointXYZ");
    defineSurfaceMovingLeastSquaresOMP<PointNormal, PointXYZRGB>(sub_module_MovingLeastSquaresOMP, "PointNormal_PointXYZRGB");
    defineSurfaceMovingLeastSquaresOMP<PointNormal, PointXYZRGBA>(sub_module_MovingLeastSquaresOMP, "PointNormal_PointXYZRGBA");
    defineSurfaceMovingLeastSquaresOMP<PointNormal, PointXYZRGBNormal>(sub_module_MovingLeastSquaresOMP, "PointNormal_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquaresOMP<PointXYZ, PointNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZ_PointNormal");
    defineSurfaceMovingLeastSquaresOMP<PointXYZ, PointXYZ>(sub_module_MovingLeastSquaresOMP, "PointXYZ_PointXYZ");
    defineSurfaceMovingLeastSquaresOMP<PointXYZ, PointXYZRGB>(sub_module_MovingLeastSquaresOMP, "PointXYZ_PointXYZRGB");
    defineSurfaceMovingLeastSquaresOMP<PointXYZ, PointXYZRGBA>(sub_module_MovingLeastSquaresOMP, "PointXYZ_PointXYZRGBA");
    defineSurfaceMovingLeastSquaresOMP<PointXYZ, PointXYZRGBNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZ_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGB, PointNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGB_PointNormal");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGB, PointXYZ>(sub_module_MovingLeastSquaresOMP, "PointXYZRGB_PointXYZ");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGB, PointXYZRGB>(sub_module_MovingLeastSquaresOMP, "PointXYZRGB_PointXYZRGB");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGB, PointXYZRGBA>(sub_module_MovingLeastSquaresOMP, "PointXYZRGB_PointXYZRGBA");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGB, PointXYZRGBNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGB_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGBA, PointNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBA_PointNormal");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGBA, PointXYZ>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBA_PointXYZ");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGBA, PointXYZRGB>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBA_PointXYZRGB");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGBA, PointXYZRGBA>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBA_PointXYZRGBA");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGBA, PointXYZRGBNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBA_PointXYZRGBNormal");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGBNormal, PointNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBNormal_PointNormal");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGBNormal, PointXYZ>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBNormal_PointXYZ");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGBNormal, PointXYZRGB>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBNormal_PointXYZRGB");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGBNormal, PointXYZRGBA>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBNormal_PointXYZRGBA");
    defineSurfaceMovingLeastSquaresOMP<PointXYZRGBNormal, PointXYZRGBNormal>(sub_module_MovingLeastSquaresOMP, "PointXYZRGBNormal_PointXYZRGBNormal");
}