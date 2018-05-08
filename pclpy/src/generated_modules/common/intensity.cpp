
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

#include <pcl/common/intensity.h>

using namespace pcl::common;


template<typename PointT>
void defineCommonIntensityFieldAccessor(py::module &m, std::string const & suffix) {
    using Class = pcl::common::IntensityFieldAccessor<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    // Operators not implemented (operator());
    cls.def("demean", &Class::demean, "p"_a, "value"_a);
    cls.def("add", &Class::add, "p"_a, "value"_a);
    cls.def("set", &Class::set, "p"_a, "intensity"_a);
    cls.def("get", &Class::get, "p"_a, "intensity"_a);
        
}

void defineCommonIntensityFunctions(py::module &m) {
}

void defineCommonIntensityClasses(py::module &sub_module) {
}