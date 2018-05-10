#pragma warning(disable : 4506)
#include <pcl/features/grsd.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesGRSDEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::GRSDEstimation<PointInT, PointNT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setRadiusSearch", &Class::setRadiusSearch, "radius"_a);
    cls.def("getRadiusSearch", &Class::getRadiusSearch);
    cls.def_static("getSimpleType", &Class::getSimpleType, "min_radius"_a, "max_radius"_a, "min_radius_plane"_a=0.100, "max_radius_noise"_a=0.015, "min_radius_cylinder"_a=0.175, "max_min_radius_diff"_a=0.050);
        
}

void defineFeaturesGrsdFunctions(py::module &m) {
}

void defineFeaturesGrsdClasses(py::module &sub_module) {
    py::module sub_module_GRSDEstimation = sub_module.def_submodule("GRSDEstimation", "Submodule GRSDEstimation");
    defineFeaturesGRSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::GRSDSignature21>(sub_module_GRSDEstimation, "PointXYZ_Normal_GRSDSignature21");
    defineFeaturesGRSDEstimation<pcl::PointXYZI, pcl::Normal, pcl::GRSDSignature21>(sub_module_GRSDEstimation, "PointXYZI_Normal_GRSDSignature21");
    defineFeaturesGRSDEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::GRSDSignature21>(sub_module_GRSDEstimation, "PointXYZRGBA_Normal_GRSDSignature21");
}