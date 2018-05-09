
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/shot_lrf.h>



template<typename PointInT, typename PointOutT = ReferenceFrame>
void defineFeaturesSHOTLocalReferenceFrameEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineFeaturesShotLrfFunctions(py::module &m) {
}

void defineFeaturesShotLrfClasses(py::module &sub_module) {
    py::module sub_module_SHOTLocalReferenceFrameEstimation = sub_module.def_submodule("SHOTLocalReferenceFrameEstimation", "Submodule SHOTLocalReferenceFrameEstimation");
    defineFeaturesSHOTLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimation, "PointXYZ_ReferenceFrame");
    defineFeaturesSHOTLocalReferenceFrameEstimation<pcl::PointXYZI, pcl::ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimation, "PointXYZI_ReferenceFrame");
    defineFeaturesSHOTLocalReferenceFrameEstimation<pcl::PointXYZRGB, pcl::ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimation, "PointXYZRGB_ReferenceFrame");
    defineFeaturesSHOTLocalReferenceFrameEstimation<pcl::PointXYZRGBA, pcl::ReferenceFrame>(sub_module_SHOTLocalReferenceFrameEstimation, "PointXYZRGBA_ReferenceFrame");
}