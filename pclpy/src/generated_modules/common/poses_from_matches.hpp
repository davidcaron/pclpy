
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/poses_from_matches.h>



void defineCommonPosesFromMatches(py::module &m) {
    using Class = PosesFromMatches;
    using PoseEstimatesVector = Class::PoseEstimatesVector;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PosesFromMatches");
    cls.def(py::init<>());
    cls.def("estimate_poses_using1_correspondence", &Class::estimatePosesUsing1Correspondence);
    cls.def("estimate_poses_using2_correspondences", &Class::estimatePosesUsing2Correspondences);
    cls.def("estimate_poses_using3_correspondences", &Class::estimatePosesUsing3Correspondences);
}

void defineCommonPosesFromMatchesClasses(py::module &sub_module) {
    defineCommonPosesFromMatches(sub_module);
}