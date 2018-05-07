
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/filters/covariance_sampling.h>



template <typename PointT, typename PointNT>
void defineFiltersCovarianceSampling(py::module &m, std::string const & suffix) {
    using Class = pcl::CovarianceSampling<PointT, PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computeConditionNumber", py::overload_cast<> (&Class::computeConditionNumber));
    cls.def("computeCovarianceMatrix", &Class::computeCovarianceMatrix, "covariance_matrix"_a);
    cls.def("setNumberOfSamples", &Class::setNumberOfSamples, "samples"_a);
    cls.def("setNormals", &Class::setNormals, "normals"_a);
    cls.def("getNumberOfSamples", &Class::getNumberOfSamples);
    cls.def("getNormals", &Class::getNormals);
        
}

void defineFiltersCovarianceSamplingFunctions(py::module &m) {
}

void defineFiltersCovarianceSamplingClasses(py::module &sub_module) {
    py::module sub_module_CovarianceSampling = sub_module.def_submodule("CovarianceSampling", "Submodule CovarianceSampling");
    defineFiltersCovarianceSampling<pcl::PointNormal, pcl::Normal>(sub_module_CovarianceSampling, "PointNormal_Normal");
    defineFiltersCovarianceSampling<pcl::PointNormal, pcl::PointNormal>(sub_module_CovarianceSampling, "PointNormal_PointNormal");
    defineFiltersCovarianceSampling<pcl::PointNormal, pcl::PointXYZRGBNormal>(sub_module_CovarianceSampling, "PointNormal_PointXYZRGBNormal");
    defineFiltersCovarianceSampling<pcl::PointXYZ, pcl::Normal>(sub_module_CovarianceSampling, "PointXYZ_Normal");
    defineFiltersCovarianceSampling<pcl::PointXYZ, pcl::PointNormal>(sub_module_CovarianceSampling, "PointXYZ_PointNormal");
    defineFiltersCovarianceSampling<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_CovarianceSampling, "PointXYZ_PointXYZRGBNormal");
    defineFiltersCovarianceSampling<pcl::PointXYZRGB, pcl::Normal>(sub_module_CovarianceSampling, "PointXYZRGB_Normal");
    defineFiltersCovarianceSampling<pcl::PointXYZRGB, pcl::PointNormal>(sub_module_CovarianceSampling, "PointXYZRGB_PointNormal");
    defineFiltersCovarianceSampling<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_CovarianceSampling, "PointXYZRGB_PointXYZRGBNormal");
    defineFiltersCovarianceSampling<pcl::PointXYZRGBA, pcl::Normal>(sub_module_CovarianceSampling, "PointXYZRGBA_Normal");
    defineFiltersCovarianceSampling<pcl::PointXYZRGBA, pcl::PointNormal>(sub_module_CovarianceSampling, "PointXYZRGBA_PointNormal");
    defineFiltersCovarianceSampling<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_CovarianceSampling, "PointXYZRGBA_PointXYZRGBNormal");
    defineFiltersCovarianceSampling<pcl::PointXYZRGBNormal, pcl::Normal>(sub_module_CovarianceSampling, "PointXYZRGBNormal_Normal");
    defineFiltersCovarianceSampling<pcl::PointXYZRGBNormal, pcl::PointNormal>(sub_module_CovarianceSampling, "PointXYZRGBNormal_PointNormal");
    defineFiltersCovarianceSampling<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(sub_module_CovarianceSampling, "PointXYZRGBNormal_PointXYZRGBNormal");
}