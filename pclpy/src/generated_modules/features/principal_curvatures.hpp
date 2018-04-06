
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/principal_curvatures.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::PrincipalCurvatures>
void defineFeaturesPrincipalCurvaturesEstimation(py::module &m, std::string const & suffix) {
    using Class = PrincipalCurvaturesEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute_point_principal_curvatures", &Class::computePointPrincipalCurvatures);
        
}

void defineFeaturesPrincipalCurvaturesClasses(py::module &sub_module) {
    py::module sub_module_PrincipalCurvaturesEstimation = sub_module.def_submodule("PrincipalCurvaturesEstimation", "Submodule PrincipalCurvaturesEstimation");
    defineFeaturesPrincipalCurvaturesEstimation<PointXYZ, Normal, PrincipalCurvatures>(sub_module_PrincipalCurvaturesEstimation, "PointXYZ_Normal_PrincipalCurvatures");
    defineFeaturesPrincipalCurvaturesEstimation<PointXYZI, Normal, PrincipalCurvatures>(sub_module_PrincipalCurvaturesEstimation, "PointXYZI_Normal_PrincipalCurvatures");
    defineFeaturesPrincipalCurvaturesEstimation<PointXYZRGBA, Normal, PrincipalCurvatures>(sub_module_PrincipalCurvaturesEstimation, "PointXYZRGBA_Normal_PrincipalCurvatures");
}