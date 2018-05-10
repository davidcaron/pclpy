
#include <pcl/features/normal_3d_omp.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesNormalEstimationOMP(py::module &m, std::string const & suffix) {
    using Class = pcl::NormalEstimationOMP<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::NormalEstimation<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "nr_threads"_a=0);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineFeaturesNormal3dOmpFunctions(py::module &m) {
}

void defineFeaturesNormal3dOmpClasses(py::module &sub_module) {
    py::module sub_module_NormalEstimationOMP = sub_module.def_submodule("NormalEstimationOMP", "Submodule NormalEstimationOMP");
    defineFeaturesNormalEstimationOMP<pcl::PointNormal, pcl::Normal>(sub_module_NormalEstimationOMP, "PointNormal_Normal");
    defineFeaturesNormalEstimationOMP<pcl::PointNormal, pcl::PointNormal>(sub_module_NormalEstimationOMP, "PointNormal_PointNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointNormal, pcl::PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointNormal_PointXYZRGBNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointSurfel, pcl::Normal>(sub_module_NormalEstimationOMP, "PointSurfel_Normal");
    defineFeaturesNormalEstimationOMP<pcl::PointSurfel, pcl::PointNormal>(sub_module_NormalEstimationOMP, "PointSurfel_PointNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointSurfel, pcl::PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointSurfel_PointXYZRGBNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZ, pcl::Normal>(sub_module_NormalEstimationOMP, "PointXYZ_Normal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal>(sub_module_NormalEstimationOMP, "PointXYZ_PointNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointXYZ_PointXYZRGBNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZI, pcl::Normal>(sub_module_NormalEstimationOMP, "PointXYZI_Normal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZI, pcl::PointNormal>(sub_module_NormalEstimationOMP, "PointXYZI_PointNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZI, pcl::PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointXYZI_PointXYZRGBNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal>(sub_module_NormalEstimationOMP, "PointXYZRGB_Normal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal>(sub_module_NormalEstimationOMP, "PointXYZRGB_PointNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointXYZRGB_PointXYZRGBNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal>(sub_module_NormalEstimationOMP, "PointXYZRGBA_Normal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZRGBA, pcl::PointNormal>(sub_module_NormalEstimationOMP, "PointXYZRGBA_PointNormal");
    defineFeaturesNormalEstimationOMP<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointXYZRGBA_PointXYZRGBNormal");
}