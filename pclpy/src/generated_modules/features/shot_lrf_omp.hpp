
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/shot_lrf_omp.h>



template<typename PointInT, typename PointOutT = ReferenceFrame>
void defineFeaturesSHOTLocalReferenceFrameEstimationOMP(py::module &m, std::string const & suffix) {
    using Class = SHOTLocalReferenceFrameEstimationOMP<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, SHOTLocalReferenceFrameEstimation<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineFeaturesShotLrfOmpClasses(py::module &sub_module) {
    py::module sub_module_SHOTLocalReferenceFrameEstimationOMP = sub_module.def_submodule("SHOTLocalReferenceFrameEstimationOMP", "Submodule SHOTLocalReferenceFrameEstimationOMP");
    defineFeaturesSHOTLocalReferenceFrameEstimationOMP<PointXYZ, ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimationOMP, "PointXYZ_ReferenceFrame");
    defineFeaturesSHOTLocalReferenceFrameEstimationOMP<PointXYZI, ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimationOMP, "PointXYZI_ReferenceFrame");
    defineFeaturesSHOTLocalReferenceFrameEstimationOMP<PointXYZRGB, ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimationOMP, "PointXYZRGB_ReferenceFrame");
    defineFeaturesSHOTLocalReferenceFrameEstimationOMP<PointXYZRGBA, ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimationOMP, "PointXYZRGBA_ReferenceFrame");
}