
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/shot_omp.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT1344, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesSHOTColorEstimationOMP(py::module &m, std::string const & suffix) {
    using Class = SHOTColorEstimationOMP<PointInT, PointNT, PointOutT, PointRFT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, SHOTColorEstimation<PointInT,PointNT,PointOutT,PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT352, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesSHOTEstimationOMP(py::module &m, std::string const & suffix) {
    using Class = SHOTEstimationOMP<PointInT, PointNT, PointOutT, PointRFT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, SHOTEstimation<PointInT,PointNT,PointOutT,PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineFeaturesShotOmpClasses(py::module &sub_module) {
    py::module sub_module_SHOTColorEstimationOMP = sub_module.def_submodule("SHOTColorEstimationOMP", "Submodule SHOTColorEstimationOMP");
    defineFeaturesSHOTColorEstimationOMP<PointXYZRGB, Normal, SHOT1344, ReferenceFrame>(sub_module_SHOTColorEstimationOMP, "PointXYZRGB_Normal_SHOT1344_ReferenceFrame");
    defineFeaturesSHOTColorEstimationOMP<PointXYZRGBA, Normal, SHOT1344, ReferenceFrame>(sub_module_SHOTColorEstimationOMP, "PointXYZRGBA_Normal_SHOT1344_ReferenceFrame");
    py::module sub_module_SHOTEstimationOMP = sub_module.def_submodule("SHOTEstimationOMP", "Submodule SHOTEstimationOMP");
    defineFeaturesSHOTEstimationOMP<PointXYZ, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimationOMP, "PointXYZ_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationOMP<PointXYZI, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimationOMP, "PointXYZI_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationOMP<PointXYZRGB, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimationOMP, "PointXYZRGB_Normal_SHOT352_ReferenceFrame");
    defineFeaturesSHOTEstimationOMP<PointXYZRGBA, Normal, SHOT352, ReferenceFrame>(sub_module_SHOTEstimationOMP, "PointXYZRGBA_Normal_SHOT352_ReferenceFrame");
}