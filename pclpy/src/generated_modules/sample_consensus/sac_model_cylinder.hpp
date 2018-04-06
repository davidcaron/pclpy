
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_cylinder.h>



template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelCylinder(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelCylinder<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, SampleConsensusModel<PointT>, SampleConsensusModelFromNormals<PointT,PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<Class::PointCloudConstPtr, bool>(), "cloud"_a, "random"_a=false);
    cls.def(py::init<Class::PointCloudConstPtr, std::vector<int>, bool>(), "cloud"_a, "indices"_a, "random"_a=false);
    cls.def_property("eps_angle", &Class::getEpsAngle, &Class::setEpsAngle);
    cls.def_property("axis", &Class::getAxis, &Class::setAxis);
    // Operators not implemented (operator=);
    cls.def("compute_model_coefficients", &Class::computeModelCoefficients);
    cls.def("select_within_distance", &Class::selectWithinDistance);
    cls.def("count_within_distance", &Class::countWithinDistance);
    cls.def("optimize_model_coefficients", &Class::optimizeModelCoefficients);
    cls.def("project_points", &Class::projectPoints);
    cls.def("do_samples_verify_model", &Class::doSamplesVerifyModel);
        
}

void defineSampleConsensusSacModelCylinderClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelCylinder = sub_module.def_submodule("SampleConsensusModelCylinder", "Submodule SampleConsensusModelCylinder");
    defineSampleConsensusSampleConsensusModelCylinder<PointXYZ, Normal>(sub_module_SampleConsensusModelCylinder, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelCylinder<PointXYZI, Normal>(sub_module_SampleConsensusModelCylinder, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelCylinder<PointXYZRGB, Normal>(sub_module_SampleConsensusModelCylinder, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelCylinder<PointXYZRGBA, Normal>(sub_module_SampleConsensusModelCylinder, "PointXYZRGBA_Normal");
}