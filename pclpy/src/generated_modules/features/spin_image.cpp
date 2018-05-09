
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/spin_image.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesSpinImageEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::SpinImageEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int, double, unsigned int>(), "image_width"_a=8, "support_angle_cos"_a=0.0, "min_pts_neighb"_a=0);
    cls.def("useNormalsAsRotationAxis", &Class::useNormalsAsRotationAxis);
    cls.def("setImageWidth", &Class::setImageWidth, "bin_count"_a);
    cls.def("setSupportAngle", &Class::setSupportAngle, "support_angle_cos"_a);
    cls.def("setMinPointCountInNeighbourhood", &Class::setMinPointCountInNeighbourhood, "min_pts_neighb"_a);
    cls.def("setInputNormals", &Class::setInputNormals, "normals"_a);
    cls.def("setRotationAxis", &Class::setRotationAxis, "axis"_a);
    cls.def("setInputRotationAxes", &Class::setInputRotationAxes, "axes"_a);
    cls.def("setAngularDomain", &Class::setAngularDomain, "is_angular"_a=true);
    cls.def("setRadialStructure", &Class::setRadialStructure, "is_radial"_a=true);
        
}

void defineFeaturesSpinImageFunctions(py::module &m) {
}

void defineFeaturesSpinImageClasses(py::module &sub_module) {
    py::module sub_module_SpinImageEstimation = sub_module.def_submodule("SpinImageEstimation", "Submodule SpinImageEstimation");
    defineFeaturesSpinImageEstimation<pcl::PointNormal, pcl::Normal, pcl::Histogram<153>>(sub_module_SpinImageEstimation, "PointNormal_Normal_Histogram<153>");
    defineFeaturesSpinImageEstimation<pcl::PointNormal, pcl::PointNormal, pcl::Histogram<153>>(sub_module_SpinImageEstimation, "PointNormal_PointNormal_Histogram<153>");
    defineFeaturesSpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZ_Normal_Histogram<153>");
    defineFeaturesSpinImageEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZ_PointNormal_Histogram<153>");
    defineFeaturesSpinImageEstimation<pcl::PointXYZI, pcl::Normal, pcl::Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZI_Normal_Histogram<153>");
    defineFeaturesSpinImageEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZI_PointNormal_Histogram<153>");
    defineFeaturesSpinImageEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZRGBA_Normal_Histogram<153>");
    defineFeaturesSpinImageEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZRGBA_PointNormal_Histogram<153>");
}