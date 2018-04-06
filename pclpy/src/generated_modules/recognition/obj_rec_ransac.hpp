
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/obj_rec_ransac.h>

using namespace pcl::recognition;


void defineRecognitionObjRecRANSAC(py::module &m) {
    using Class = recognition::ObjRecRANSAC;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudN = Class::PointCloudN;
    using BVHH = Class::BVHH;
    using HypothesisOctree = Class::HypothesisOctree;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "ObjRecRANSAC");
    cls.def(py::init<float, float>(), "pair_width"_a, "voxel_size"_a);
    cls.def("set_max_coplanarity_angle_degrees", &Class::setMaxCoplanarityAngleDegrees);
    cls.def("set_scene_bounds_enlargement_factor", &Class::setSceneBoundsEnlargementFactor);
    cls.def("clear", &Class::clear);
    cls.def("ignore_coplanar_point_pairs_on", &Class::ignoreCoplanarPointPairsOn);
    cls.def("ignore_coplanar_point_pairs_off", &Class::ignoreCoplanarPointPairsOff);
    cls.def("icp_hypotheses_refinement_on", &Class::icpHypothesesRefinementOn);
    cls.def("icp_hypotheses_refinement_off", &Class::icpHypothesesRefinementOff);
    cls.def("add_model", &Class::addModel);
    cls.def("recognize", &Class::recognize);
    cls.def("enter_test_mode_sample_opp", &Class::enterTestModeSampleOPP);
    cls.def("enter_test_mode_test_hypotheses", &Class::enterTestModeTestHypotheses);
    cls.def("leave_test_mode", &Class::leaveTestMode);
    cls.def("get_accepted_hypotheses", py::overload_cast<> (&Class::getAcceptedHypotheses, py::const_));
    cls.def("get_accepted_hypotheses", py::overload_cast<std::vector<Hypothesis> &> (&Class::getAcceptedHypotheses, py::const_));
}

void defineRecognitionObjRecRansacClasses(py::module &sub_module) {
    defineRecognitionObjRecRANSAC(sub_module);
}