
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/moment_invariants.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesMomentInvariantsEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::MomentInvariantsEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computePointMomentInvariants", py::overload_cast<const pcl::PointCloud<PointInT> &, const std::vector<int> &, float &, float &, float &> (&Class::computePointMomentInvariants), "cloud"_a, "indices"_a, "j1"_a, "j2"_a, "j3"_a);
    cls.def("computePointMomentInvariants", py::overload_cast<const pcl::PointCloud<PointInT> &, float &, float &, float &> (&Class::computePointMomentInvariants), "cloud"_a, "j1"_a, "j2"_a, "j3"_a);
        
}

void defineFeaturesMomentInvariantsFunctions(py::module &m) {
}

void defineFeaturesMomentInvariantsClasses(py::module &sub_module) {
    py::module sub_module_MomentInvariantsEstimation = sub_module.def_submodule("MomentInvariantsEstimation", "Submodule MomentInvariantsEstimation");
    defineFeaturesMomentInvariantsEstimation<pcl::PointXYZ, pcl::MomentInvariants>(sub_module_MomentInvariantsEstimation, "PointXYZ_MomentInvariants");
    defineFeaturesMomentInvariantsEstimation<pcl::PointXYZI, pcl::MomentInvariants>(sub_module_MomentInvariantsEstimation, "PointXYZI_MomentInvariants");
    defineFeaturesMomentInvariantsEstimation<pcl::PointXYZRGBA, pcl::MomentInvariants>(sub_module_MomentInvariantsEstimation, "PointXYZRGBA_MomentInvariants");
}