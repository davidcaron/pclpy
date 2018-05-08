
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

#include <pcl/features/esf.h>



template <typename PointInT,  typename PointOutT = pcl::ESFSignature640>
void defineFeaturesESFEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::ESFEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute", &Class::compute, "output"_a);
        
}

void defineFeaturesEsfFunctions(py::module &m) {
}

void defineFeaturesEsfClasses(py::module &sub_module) {
    py::module sub_module_ESFEstimation = sub_module.def_submodule("ESFEstimation", "Submodule ESFEstimation");
    defineFeaturesESFEstimation<pcl::PointNormal, pcl::ESFSignature640>(sub_module_ESFEstimation, "PointNormal_ESFSignature640");
    defineFeaturesESFEstimation<pcl::PointXYZ, pcl::ESFSignature640>(sub_module_ESFEstimation, "PointXYZ_ESFSignature640");
    defineFeaturesESFEstimation<pcl::PointXYZI, pcl::ESFSignature640>(sub_module_ESFEstimation, "PointXYZI_ESFSignature640");
    defineFeaturesESFEstimation<pcl::PointXYZRGBA, pcl::ESFSignature640>(sub_module_ESFEstimation, "PointXYZRGBA_ESFSignature640");
}