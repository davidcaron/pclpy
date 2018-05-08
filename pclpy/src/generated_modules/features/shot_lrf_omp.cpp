
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

#include <pcl/features/shot_lrf_omp.h>



template<typename PointInT, typename PointOutT = ReferenceFrame>
void defineFeaturesSHOTLocalReferenceFrameEstimationOMP(py::module &m, std::string const & suffix) {
    using Class = pcl::SHOTLocalReferenceFrameEstimationOMP<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineFeaturesShotLrfOmpFunctions(py::module &m) {
}

void defineFeaturesShotLrfOmpClasses(py::module &sub_module) {
    py::module sub_module_SHOTLocalReferenceFrameEstimationOMP = sub_module.def_submodule("SHOTLocalReferenceFrameEstimationOMP", "Submodule SHOTLocalReferenceFrameEstimationOMP");
    defineFeaturesSHOTLocalReferenceFrameEstimationOMP<pcl::PointXYZ, pcl::ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimationOMP, "PointXYZ_ReferenceFrame");
    defineFeaturesSHOTLocalReferenceFrameEstimationOMP<pcl::PointXYZI, pcl::ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimationOMP, "PointXYZI_ReferenceFrame");
    defineFeaturesSHOTLocalReferenceFrameEstimationOMP<pcl::PointXYZRGB, pcl::ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimationOMP, "PointXYZRGB_ReferenceFrame");
    defineFeaturesSHOTLocalReferenceFrameEstimationOMP<pcl::PointXYZRGBA, pcl::ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimationOMP, "PointXYZRGBA_ReferenceFrame");
}