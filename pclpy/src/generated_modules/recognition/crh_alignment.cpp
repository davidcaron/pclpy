
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

#include <pcl/recognition/crh_alignment.h>



template<typename PointT, int nbins_>
void defineRecognitionCRHAlignment(py::module &m, std::string const & suffix) {
    using Class = pcl::CRHAlignment<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("align", &Class::align, "input_ftt"_a, "target_ftt"_a);
    cls.def("computeRollAngle", &Class::computeRollAngle, "input_ftt"_a, "target_ftt"_a, "peaks"_a);
    cls.def("setInputAndTargetView", &Class::setInputAndTargetView, "input_view"_a, "target_view"_a);
    cls.def("setInputAndTargetCentroids", &Class::setInputAndTargetCentroids, "c1"_a, "c2"_a);
    cls.def("getTransforms", &Class::getTransforms, "transforms"_a);
        
}

void defineRecognitionCrhAlignmentFunctions(py::module &m) {
}

void defineRecognitionCrhAlignmentClasses(py::module &sub_module) {
}