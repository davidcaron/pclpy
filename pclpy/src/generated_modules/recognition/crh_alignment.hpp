
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/crh_alignment.h>



template<typename PointT, int nbins_>
void defineRecognitionCRHAlignment(py::module &m, std::string const & suffix) {
    using Class = CRHAlignment<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_input_and_target_view", &Class::setInputAndTargetView);
    cls.def("set_input_and_target_centroids", &Class::setInputAndTargetCentroids);
    cls.def("align", &Class::align);
    cls.def("compute_roll_angle", &Class::computeRollAngle);
        
}

void defineRecognitionCrhAlignmentClasses(py::module &sub_module) {
}