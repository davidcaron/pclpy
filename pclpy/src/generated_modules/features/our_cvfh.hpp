
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/our_cvfh.h>



template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
void defineFeaturesOURCVFHEstimation(py::module &m, std::string const & suffix) {
    using Class = OURCVFHEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using KdTreePtr = Class::KdTreePtr;
    using PointInTPtr = Class::PointInTPtr;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("view_point", &Class::getViewPoint, &Class::setViewPoint);
    cls.def("set_radius_normals", &Class::setRadiusNormals);
    cls.def("set_cluster_tolerance", &Class::setClusterTolerance);
    cls.def("set_eps_angle_threshold", &Class::setEPSAngleThreshold);
    cls.def("set_curvature_threshold", &Class::setCurvatureThreshold);
    cls.def("set_min_points", &Class::setMinPoints);
    cls.def("set_normalize_bins", &Class::setNormalizeBins);
    cls.def("set_refine_clusters", &Class::setRefineClusters);
    cls.def("set_axis_ratio", &Class::setAxisRatio);
    cls.def("set_min_axis_value", &Class::setMinAxisValue);
    cls.def("create_trans_from_axes", &Class::createTransFromAxes);
    cls.def("compute_rf_and_shape_distribution", &Class::computeRFAndShapeDistribution);
    cls.def("sgurf", &Class::sgurf);
    cls.def("filter_normals_with_high_curvature", &Class::filterNormalsWithHighCurvature);
    cls.def("compute", &Class::compute);
        
}

void defineFeaturesOurCvfhClasses(py::module &sub_module) {
    py::module sub_module_OURCVFHEstimation = sub_module.def_submodule("OURCVFHEstimation", "Submodule OURCVFHEstimation");
    defineFeaturesOURCVFHEstimation<PointXYZ, Normal, VFHSignature308>(sub_module_OURCVFHEstimation, "PointXYZ_Normal_VFHSignature308");
    defineFeaturesOURCVFHEstimation<PointXYZRGB, Normal, VFHSignature308>(sub_module_OURCVFHEstimation, "PointXYZRGB_Normal_VFHSignature308");
}