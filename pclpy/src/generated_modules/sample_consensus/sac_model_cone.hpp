
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/sample_consensus/sac_model_cone.h>



template <typename PointT, typename PointNT>
void defineSampleConsensusSampleConsensusModelCone(py::module &m, std::string const & suffix) {
    using Class = SampleConsensusModelCone<PointT, PointNT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    py::class_<Class, SampleConsensusModel<PointT>, SampleConsensusModelFromNormals<PointT,PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<Class::PointCloudConstPtr, bool>(), "cloud"_a, "random"_a=false);
    cls.def(py::init<Class::PointCloudConstPtr, std::vector<int>, bool>(), "cloud"_a, "indices"_a, "random"_a=false);
    cls.def_property("eps_angle", &Class::getEpsAngle, &Class::setEpsAngle);
    cls.def_property("axis", &Class::getAxis, &Class::setAxis);
    cls.def_property("min_max_opening_angle", &Class::getMinMaxOpeningAngle, &Class::setMinMaxOpeningAngle);
    // Operators not implemented (operator=);
    cls.def("compute_model_coefficients", &Class::computeModelCoefficients);
    cls.def("select_within_distance", &Class::selectWithinDistance);
    cls.def("count_within_distance", &Class::countWithinDistance);
    cls.def("optimize_model_coefficients", &Class::optimizeModelCoefficients);
    cls.def("project_points", &Class::projectPoints);
    cls.def("do_samples_verify_model", &Class::doSamplesVerifyModel);
        
}

void defineSampleConsensusSacModelConeClasses(py::module &sub_module) {
    py::module sub_module_SampleConsensusModelCone = sub_module.def_submodule("SampleConsensusModelCone", "Submodule SampleConsensusModelCone");
    defineSampleConsensusSampleConsensusModelCone<PointXYZ, Normal>(sub_module_SampleConsensusModelCone, "PointXYZ_Normal");
    defineSampleConsensusSampleConsensusModelCone<PointXYZI, Normal>(sub_module_SampleConsensusModelCone, "PointXYZI_Normal");
    defineSampleConsensusSampleConsensusModelCone<PointXYZRGB, Normal>(sub_module_SampleConsensusModelCone, "PointXYZRGB_Normal");
    defineSampleConsensusSampleConsensusModelCone<PointXYZRGBA, Normal>(sub_module_SampleConsensusModelCone, "PointXYZRGBA_Normal");
}