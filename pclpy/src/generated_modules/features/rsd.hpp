
#include <pcl/features/rsd.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesRSDEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::RSDEstimation<PointInT, PointNT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setNrSubdivisions", &Class::setNrSubdivisions, "nr_subdiv"_a);
    cls.def("setPlaneRadius", &Class::setPlaneRadius, "plane_radius"_a);
    cls.def("setKSearch", &Class::setKSearch, ""_a);
    cls.def("setSaveHistograms", &Class::setSaveHistograms, "save"_a);
    cls.def("getNrSubdivisions", &Class::getNrSubdivisions);
    cls.def("getPlaneRadius", &Class::getPlaneRadius);
    cls.def("getSaveHistograms", &Class::getSaveHistograms);
    cls.def("getHistograms", &Class::getHistograms);
        
}

void defineFeaturesRsdFunctions(py::module &m) {
}

void defineFeaturesRsdClasses(py::module &sub_module) {
    py::module sub_module_RSDEstimation = sub_module.def_submodule("RSDEstimation", "Submodule RSDEstimation");
    defineFeaturesRSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalRadiiRSD>(sub_module_RSDEstimation, "PointXYZ_Normal_PrincipalRadiiRSD");
    defineFeaturesRSDEstimation<pcl::PointXYZI, pcl::Normal, pcl::PrincipalRadiiRSD>(sub_module_RSDEstimation, "PointXYZI_Normal_PrincipalRadiiRSD");
    defineFeaturesRSDEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PrincipalRadiiRSD>(sub_module_RSDEstimation, "PointXYZRGBA_Normal_PrincipalRadiiRSD");
    defineFeaturesRsdFunctions(sub_module);
}