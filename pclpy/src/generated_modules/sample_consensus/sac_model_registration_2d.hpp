
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_registration_2d.h>



template <typename PointT>
void defineSampleConsensusSampleConsensusModelRegistration2D(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelRegistration2D<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, SampleConsensusModelRegistration<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("projection_matrix", &Class::getProjectionMatrix, &Class::setProjectionMatrix);
    cls.def("select_within_distance", &Class::selectWithinDistance);
    cls.def("count_within_distance", &Class::countWithinDistance);
        
}

void defineSampleConsensusSacModelRegistration2dClasses(py::module &sub_module) {
}