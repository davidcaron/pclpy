
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

#include <pcl/sample_consensus/sac_model_registration_2d.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelRegistration2D(py::module &m, std::string const & suffix) {
    using Class = pcl::SampleConsensusModelRegistration2D<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::SampleConsensusModelRegistration<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("selectWithinDistance", &Class::selectWithinDistance, "model_coefficients"_a, "threshold"_a, "inliers"_a);
    cls.def("countWithinDistance", &Class::countWithinDistance, "model_coefficients"_a, "threshold"_a);
    cls.def("setProjectionMatrix", &Class::setProjectionMatrix, "projection_matrix"_a);
    cls.def("getDistancesToModel", &Class::getDistancesToModel, "model_coefficients"_a, "distances"_a);
    cls.def("getProjectionMatrix", &Class::getProjectionMatrix);
        
}

void defineSampleConsensusSacModelRegistration2dFunctions(py::module &m) {
}

void defineSampleConsensusSacModelRegistration2dClasses(py::module &sub_module) {
}