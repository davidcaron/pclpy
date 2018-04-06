
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/board.h>



template<typename PointInT, typename PointNT, typename PointOutT = ReferenceFrame>
void defineFeaturesBOARDLocalReferenceFrameEstimation(py::module &m, std::string const & suffix) {
    using Class = BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("tangent_radius", &Class::getTangentRadius, &Class::setTangentRadius);
    cls.def_property("find_holes", &Class::getFindHoles, &Class::setFindHoles);
    cls.def_property("margin_thresh", &Class::getMarginThresh, &Class::setMarginThresh);
    cls.def_property("check_margin_array_size", &Class::getCheckMarginArraySize, &Class::setCheckMarginArraySize);
    cls.def_property("hole_size_prob_thresh", &Class::getHoleSizeProbThresh, &Class::setHoleSizeProbThresh);
    cls.def_property("steep_thresh", &Class::getSteepThresh, &Class::setSteepThresh);
        
}

void defineFeaturesBoardClasses(py::module &sub_module) {
    py::module sub_module_BOARDLocalReferenceFrameEstimation = sub_module.def_submodule("BOARDLocalReferenceFrameEstimation", "Submodule BOARDLocalReferenceFrameEstimation");
    defineFeaturesBOARDLocalReferenceFrameEstimation<PointXYZ, Normal, ReferenceFrame>(sub_module_BOARDLocalReferenceFrameEstimation, "PointXYZ_Normal_ReferenceFrame");
    defineFeaturesBOARDLocalReferenceFrameEstimation<PointXYZI, Normal, ReferenceFrame>(sub_module_BOARDLocalReferenceFrameEstimation, "PointXYZI_Normal_ReferenceFrame");
    defineFeaturesBOARDLocalReferenceFrameEstimation<PointXYZRGB, Normal, ReferenceFrame>(sub_module_BOARDLocalReferenceFrameEstimation, "PointXYZRGB_Normal_ReferenceFrame");
    defineFeaturesBOARDLocalReferenceFrameEstimation<PointXYZRGBA, Normal, ReferenceFrame>(sub_module_BOARDLocalReferenceFrameEstimation, "PointXYZRGBA_Normal_ReferenceFrame");
}