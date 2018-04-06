
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/spin_image.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesSpinImageEstimation(py::module &m, std::string const & suffix) {
    using Class = SpinImageEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int, double, unsigned int>(), "image_width"_a=8, "support_angle_cos"_a=0.0, "min_pts_neighb"_a=0);
    cls.def("set_image_width", &Class::setImageWidth);
    cls.def("set_support_angle", &Class::setSupportAngle);
    cls.def("set_min_point_count_in_neighbourhood", &Class::setMinPointCountInNeighbourhood);
    cls.def("set_input_normals", &Class::setInputNormals);
    cls.def("set_rotation_axis", &Class::setRotationAxis);
    cls.def("set_input_rotation_axes", &Class::setInputRotationAxes);
    cls.def("set_angular_domain", &Class::setAngularDomain);
    cls.def("set_radial_structure", &Class::setRadialStructure);
    cls.def("use_normals_as_rotation_axis", &Class::useNormalsAsRotationAxis);
        
}

void defineFeaturesSpinImageClasses(py::module &sub_module) {
    py::module sub_module_SpinImageEstimation = sub_module.def_submodule("SpinImageEstimation", "Submodule SpinImageEstimation");
    defineFeaturesSpinImageEstimation<PointNormal, Normal, Histogram<153>>(sub_module_SpinImageEstimation, "PointNormal_Normal_Histogram<153>");
    defineFeaturesSpinImageEstimation<PointNormal, PointNormal, Histogram<153>>(sub_module_SpinImageEstimation, "PointNormal_PointNormal_Histogram<153>");
    defineFeaturesSpinImageEstimation<PointXYZ, Normal, Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZ_Normal_Histogram<153>");
    defineFeaturesSpinImageEstimation<PointXYZ, PointNormal, Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZ_PointNormal_Histogram<153>");
    defineFeaturesSpinImageEstimation<PointXYZI, Normal, Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZI_Normal_Histogram<153>");
    defineFeaturesSpinImageEstimation<PointXYZI, PointNormal, Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZI_PointNormal_Histogram<153>");
    defineFeaturesSpinImageEstimation<PointXYZRGBA, Normal, Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZRGBA_Normal_Histogram<153>");
    defineFeaturesSpinImageEstimation<PointXYZRGBA, PointNormal, Histogram<153>>(sub_module_SpinImageEstimation, "PointXYZRGBA_PointNormal_Histogram<153>");
}