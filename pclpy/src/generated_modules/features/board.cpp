
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

#include <pcl/features/board.h>



template<typename PointInT, typename PointNT, typename PointOutT = ReferenceFrame>
void defineFeaturesBOARDLocalReferenceFrameEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setTangentRadius", &Class::setTangentRadius, "radius"_a);
    cls.def("setFindHoles", &Class::setFindHoles, "find_holes"_a);
    cls.def("setMarginThresh", &Class::setMarginThresh, "margin_thresh"_a);
    cls.def("setCheckMarginArraySize", &Class::setCheckMarginArraySize, "size"_a);
    cls.def("setHoleSizeProbThresh", &Class::setHoleSizeProbThresh, "prob_thresh"_a);
    cls.def("setSteepThresh", &Class::setSteepThresh, "steep_thresh"_a);
    cls.def("getTangentRadius", &Class::getTangentRadius);
    cls.def("getFindHoles", &Class::getFindHoles);
    cls.def("getMarginThresh", &Class::getMarginThresh);
    cls.def("getCheckMarginArraySize", &Class::getCheckMarginArraySize);
    cls.def("getHoleSizeProbThresh", &Class::getHoleSizeProbThresh);
    cls.def("getSteepThresh", &Class::getSteepThresh);
        
}

void defineFeaturesBoardFunctions(py::module &m) {
}

void defineFeaturesBoardClasses(py::module &sub_module) {
    py::module sub_module_BOARDLocalReferenceFrameEstimation = sub_module.def_submodule("BOARDLocalReferenceFrameEstimation", "Submodule BOARDLocalReferenceFrameEstimation");
    defineFeaturesBOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame>(sub_module_BOARDLocalReferenceFrameEstimation, "PointXYZ_Normal_ReferenceFrame");
    defineFeaturesBOARDLocalReferenceFrameEstimation<pcl::PointXYZI, pcl::Normal, pcl::ReferenceFrame>(sub_module_BOARDLocalReferenceFrameEstimation, "PointXYZI_Normal_ReferenceFrame");
    defineFeaturesBOARDLocalReferenceFrameEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::ReferenceFrame>(sub_module_BOARDLocalReferenceFrameEstimation, "PointXYZRGB_Normal_ReferenceFrame");
    defineFeaturesBOARDLocalReferenceFrameEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::ReferenceFrame>(sub_module_BOARDLocalReferenceFrameEstimation, "PointXYZRGBA_Normal_ReferenceFrame");
}