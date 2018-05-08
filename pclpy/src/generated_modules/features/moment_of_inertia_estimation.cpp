
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

#include <pcl/features/moment_of_inertia_estimation.h>



template <typename PointT>
void defineFeaturesMomentOfInertiaEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::MomentOfInertiaEstimation<PointT>;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute", &Class::compute);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setIndices", py::overload_cast<const pcl::IndicesPtr &> (&Class::setIndices), "indices"_a);
    cls.def("setIndices", py::overload_cast<const pcl::IndicesConstPtr &> (&Class::setIndices), "indices"_a);
    cls.def("setIndices", py::overload_cast<const PointIndicesConstPtr &> (&Class::setIndices), "indices"_a);
    cls.def("setIndices", py::overload_cast<size_t, size_t, size_t, size_t> (&Class::setIndices), "row_start"_a, "col_start"_a, "nb_rows"_a, "nb_cols"_a);
    cls.def("setAngleStep", &Class::setAngleStep, "step"_a);
    cls.def("setNormalizePointMassFlag", &Class::setNormalizePointMassFlag, "need_to_normalize"_a);
    cls.def("setPointMass", &Class::setPointMass, "point_mass"_a);
    cls.def("getAngleStep", &Class::getAngleStep);
    cls.def("getNormalizePointMassFlag", &Class::getNormalizePointMassFlag);
    cls.def("getPointMass", &Class::getPointMass);
    cls.def("getAABB", &Class::getAABB, "min_point"_a, "max_point"_a);
    cls.def("getOBB", &Class::getOBB, "min_point"_a, "max_point"_a, "position"_a, "rotational_matrix"_a);
    cls.def("getEigenValues", &Class::getEigenValues, "major"_a, "middle"_a, "minor"_a);
    cls.def("getEigenVectors", &Class::getEigenVectors, "major"_a, "middle"_a, "minor"_a);
    cls.def("getMomentOfInertia", &Class::getMomentOfInertia, "moment_of_inertia"_a);
    cls.def("getEccentricity", &Class::getEccentricity, "eccentricity"_a);
    cls.def("getMassCenter", &Class::getMassCenter, "mass_center"_a);
        
}

void defineFeaturesMomentOfInertiaEstimationFunctions(py::module &m) {
}

void defineFeaturesMomentOfInertiaEstimationClasses(py::module &sub_module) {
    py::module sub_module_MomentOfInertiaEstimation = sub_module.def_submodule("MomentOfInertiaEstimation", "Submodule MomentOfInertiaEstimation");
    defineFeaturesMomentOfInertiaEstimation<pcl::PointNormal>(sub_module_MomentOfInertiaEstimation, "PointNormal");
    defineFeaturesMomentOfInertiaEstimation<pcl::PointXYZ>(sub_module_MomentOfInertiaEstimation, "PointXYZ");
    defineFeaturesMomentOfInertiaEstimation<pcl::PointXYZI>(sub_module_MomentOfInertiaEstimation, "PointXYZI");
    defineFeaturesMomentOfInertiaEstimation<pcl::PointXYZRGBA>(sub_module_MomentOfInertiaEstimation, "PointXYZRGBA");
}