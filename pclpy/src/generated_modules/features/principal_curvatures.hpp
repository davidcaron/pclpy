
#include <pcl/features/principal_curvatures.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::PrincipalCurvatures>
void defineFeaturesPrincipalCurvaturesEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::PrincipalCurvaturesEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computePointPrincipalCurvatures", &Class::computePointPrincipalCurvatures, "normals"_a, "p_idx"_a, "indices"_a, "pcx"_a, "pcy"_a, "pcz"_a, "pc1"_a, "pc2"_a);
        
}

void defineFeaturesPrincipalCurvaturesFunctions(py::module &m) {
}

void defineFeaturesPrincipalCurvaturesClasses(py::module &sub_module) {
    py::module sub_module_PrincipalCurvaturesEstimation = sub_module.def_submodule("PrincipalCurvaturesEstimation", "Submodule PrincipalCurvaturesEstimation");
    defineFeaturesPrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>(sub_module_PrincipalCurvaturesEstimation, "PointXYZ_Normal_PrincipalCurvatures");
    defineFeaturesPrincipalCurvaturesEstimation<pcl::PointXYZI, pcl::Normal, pcl::PrincipalCurvatures>(sub_module_PrincipalCurvaturesEstimation, "PointXYZI_Normal_PrincipalCurvatures");
    defineFeaturesPrincipalCurvaturesEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PrincipalCurvatures>(sub_module_PrincipalCurvaturesEstimation, "PointXYZRGBA_Normal_PrincipalCurvatures");
}