
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/normal_3d_omp.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesNormalEstimationOMP(py::module &m, std::string const & suffix) {
    using Class = NormalEstimationOMP<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, NormalEstimation<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "nr_threads"_a=0);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineFeaturesNormal3dOmpClasses(py::module &sub_module) {
    py::module sub_module_NormalEstimationOMP = sub_module.def_submodule("NormalEstimationOMP", "Submodule NormalEstimationOMP");
    defineFeaturesNormalEstimationOMP<PointNormal, Normal>(sub_module_NormalEstimationOMP, "PointNormal_Normal");
    defineFeaturesNormalEstimationOMP<PointNormal, PointNormal>(sub_module_NormalEstimationOMP, "PointNormal_PointNormal");
    defineFeaturesNormalEstimationOMP<PointNormal, PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointNormal_PointXYZRGBNormal");
    defineFeaturesNormalEstimationOMP<PointSurfel, Normal>(sub_module_NormalEstimationOMP, "PointSurfel_Normal");
    defineFeaturesNormalEstimationOMP<PointSurfel, PointNormal>(sub_module_NormalEstimationOMP, "PointSurfel_PointNormal");
    defineFeaturesNormalEstimationOMP<PointSurfel, PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointSurfel_PointXYZRGBNormal");
    defineFeaturesNormalEstimationOMP<PointXYZ, Normal>(sub_module_NormalEstimationOMP, "PointXYZ_Normal");
    defineFeaturesNormalEstimationOMP<PointXYZ, PointNormal>(sub_module_NormalEstimationOMP, "PointXYZ_PointNormal");
    defineFeaturesNormalEstimationOMP<PointXYZ, PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointXYZ_PointXYZRGBNormal");
    defineFeaturesNormalEstimationOMP<PointXYZI, Normal>(sub_module_NormalEstimationOMP, "PointXYZI_Normal");
    defineFeaturesNormalEstimationOMP<PointXYZI, PointNormal>(sub_module_NormalEstimationOMP, "PointXYZI_PointNormal");
    defineFeaturesNormalEstimationOMP<PointXYZI, PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointXYZI_PointXYZRGBNormal");
    defineFeaturesNormalEstimationOMP<PointXYZRGB, Normal>(sub_module_NormalEstimationOMP, "PointXYZRGB_Normal");
    defineFeaturesNormalEstimationOMP<PointXYZRGB, PointNormal>(sub_module_NormalEstimationOMP, "PointXYZRGB_PointNormal");
    defineFeaturesNormalEstimationOMP<PointXYZRGB, PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointXYZRGB_PointXYZRGBNormal");
    defineFeaturesNormalEstimationOMP<PointXYZRGBA, Normal>(sub_module_NormalEstimationOMP, "PointXYZRGBA_Normal");
    defineFeaturesNormalEstimationOMP<PointXYZRGBA, PointNormal>(sub_module_NormalEstimationOMP, "PointXYZRGBA_PointNormal");
    defineFeaturesNormalEstimationOMP<PointXYZRGBA, PointXYZRGBNormal>(sub_module_NormalEstimationOMP, "PointXYZRGBA_PointXYZRGBNormal");
}