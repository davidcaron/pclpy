
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

#include <pcl/features/rops_estimation.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesROPSEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::ROPSEstimation<PointInT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setNumberOfPartitionBins", &Class::setNumberOfPartitionBins, "number_of_bins"_a);
    cls.def("setNumberOfRotations", &Class::setNumberOfRotations, "number_of_rotations"_a);
    cls.def("setSupportRadius", &Class::setSupportRadius, "support_radius"_a);
    cls.def("setTriangles", &Class::setTriangles, "triangles"_a);
    cls.def("getNumberOfPartitionBins", &Class::getNumberOfPartitionBins);
    cls.def("getNumberOfRotations", &Class::getNumberOfRotations);
    cls.def("getSupportRadius", &Class::getSupportRadius);
    cls.def("getTriangles", &Class::getTriangles, "triangles"_a);
        
}

void defineFeaturesRopsEstimationFunctions(py::module &m) {
}

void defineFeaturesRopsEstimationClasses(py::module &sub_module) {
    py::module sub_module_ROPSEstimation = sub_module.def_submodule("ROPSEstimation", "Submodule ROPSEstimation");
    defineFeaturesROPSEstimation<pcl::PointNormal, pcl::Histogram<135>>(sub_module_ROPSEstimation, "PointNormal_Histogram<135>");
    defineFeaturesROPSEstimation<pcl::PointXYZ, pcl::Histogram<135>>(sub_module_ROPSEstimation, "PointXYZ_Histogram<135>");
    defineFeaturesROPSEstimation<pcl::PointXYZI, pcl::Histogram<135>>(sub_module_ROPSEstimation, "PointXYZI_Histogram<135>");
    defineFeaturesROPSEstimation<pcl::PointXYZRGBA, pcl::Histogram<135>>(sub_module_ROPSEstimation, "PointXYZRGBA_Histogram<135>");
}