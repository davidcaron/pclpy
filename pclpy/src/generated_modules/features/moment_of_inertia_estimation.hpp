
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/moment_of_inertia_estimation.h>



template <typename PointT>
void defineFeaturesMomentOfInertiaEstimation(py::module &m, std::string const & suffix) {
    using Class = MomentOfInertiaEstimation<PointT>;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def_property("angle_step", &Class::getAngleStep, &Class::setAngleStep);
    cls.def_property("normalize_point_mass_flag", &Class::getNormalizePointMassFlag, &Class::setNormalizePointMassFlag);
    cls.def_property("point_mass", &Class::getPointMass, &Class::setPointMass);
    cls.def("compute", &Class::compute);
    cls.def("set_indices", py::overload_cast<const IndicesPtr &> (&Class::setIndices));
    cls.def("set_indices", py::overload_cast<const IndicesConstPtr &> (&Class::setIndices));
    cls.def("set_indices", py::overload_cast<const PointIndicesConstPtr &> (&Class::setIndices));
    cls.def("set_indices", py::overload_cast<size_t, size_t, size_t, size_t> (&Class::setIndices));
        
}

void defineFeaturesMomentOfInertiaEstimationClasses(py::module &sub_module) {
    py::module sub_module_MomentOfInertiaEstimation = sub_module.def_submodule("MomentOfInertiaEstimation", "Submodule MomentOfInertiaEstimation");
    defineFeaturesMomentOfInertiaEstimation<PointNormal>(sub_module_MomentOfInertiaEstimation, "PointNormal");
    defineFeaturesMomentOfInertiaEstimation<PointXYZ>(sub_module_MomentOfInertiaEstimation, "PointXYZ");
    defineFeaturesMomentOfInertiaEstimation<PointXYZI>(sub_module_MomentOfInertiaEstimation, "PointXYZI");
    defineFeaturesMomentOfInertiaEstimation<PointXYZRGBA>(sub_module_MomentOfInertiaEstimation, "PointXYZRGBA");
}