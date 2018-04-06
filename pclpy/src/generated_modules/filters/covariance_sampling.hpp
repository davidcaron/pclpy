
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/covariance_sampling.h>



template <typename PointT, typename PointNT>
void defineFiltersCovarianceSampling(py::module &m, std::string const & suffix) {
    using Class = CovarianceSampling<PointT, PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("number_of_samples", &Class::getNumberOfSamples, &Class::setNumberOfSamples);
    cls.def_property("normals", &Class::getNormals, &Class::setNormals);
    cls.def("compute_condition_number", py::overload_cast<> (&Class::computeConditionNumber));
    cls.def("compute_condition_number", py::overload_cast<const Eigen::Matrix<double, 6, 6> &> (&Class::computeConditionNumber));
    cls.def("compute_covariance_matrix", &Class::computeCovarianceMatrix);
        
}

void defineFiltersCovarianceSamplingClasses(py::module &sub_module) {
    py::module sub_module_CovarianceSampling = sub_module.def_submodule("CovarianceSampling", "Submodule CovarianceSampling");
    defineFiltersCovarianceSampling<PointNormal, Normal>(sub_module_CovarianceSampling, "PointNormal_Normal");
    defineFiltersCovarianceSampling<PointNormal, PointNormal>(sub_module_CovarianceSampling, "PointNormal_PointNormal");
    defineFiltersCovarianceSampling<PointNormal, PointXYZRGBNormal>(sub_module_CovarianceSampling, "PointNormal_PointXYZRGBNormal");
    defineFiltersCovarianceSampling<PointXYZ, Normal>(sub_module_CovarianceSampling, "PointXYZ_Normal");
    defineFiltersCovarianceSampling<PointXYZ, PointNormal>(sub_module_CovarianceSampling, "PointXYZ_PointNormal");
    defineFiltersCovarianceSampling<PointXYZ, PointXYZRGBNormal>(sub_module_CovarianceSampling, "PointXYZ_PointXYZRGBNormal");
    defineFiltersCovarianceSampling<PointXYZRGB, Normal>(sub_module_CovarianceSampling, "PointXYZRGB_Normal");
    defineFiltersCovarianceSampling<PointXYZRGB, PointNormal>(sub_module_CovarianceSampling, "PointXYZRGB_PointNormal");
    defineFiltersCovarianceSampling<PointXYZRGB, PointXYZRGBNormal>(sub_module_CovarianceSampling, "PointXYZRGB_PointXYZRGBNormal");
    defineFiltersCovarianceSampling<PointXYZRGBA, Normal>(sub_module_CovarianceSampling, "PointXYZRGBA_Normal");
    defineFiltersCovarianceSampling<PointXYZRGBA, PointNormal>(sub_module_CovarianceSampling, "PointXYZRGBA_PointNormal");
    defineFiltersCovarianceSampling<PointXYZRGBA, PointXYZRGBNormal>(sub_module_CovarianceSampling, "PointXYZRGBA_PointXYZRGBNormal");
    defineFiltersCovarianceSampling<PointXYZRGBNormal, Normal>(sub_module_CovarianceSampling, "PointXYZRGBNormal_Normal");
    defineFiltersCovarianceSampling<PointXYZRGBNormal, PointNormal>(sub_module_CovarianceSampling, "PointXYZRGBNormal_PointNormal");
    defineFiltersCovarianceSampling<PointXYZRGBNormal, PointXYZRGBNormal>(sub_module_CovarianceSampling, "PointXYZRGBNormal_PointXYZRGBNormal");
}