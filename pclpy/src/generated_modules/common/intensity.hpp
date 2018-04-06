
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/intensity.h>

using namespace pcl::common;


template<typename PointT>
void defineCommonIntensityFieldAccessor(py::module &m, std::string const & suffix) {
    using Class = common::IntensityFieldAccessor<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("", &Class::get, &Class::set);
    // Operators not implemented (operator());
    cls.def("demean", &Class::demean);
    cls.def("add", &Class::add);
        
}

void defineCommonIntensityClasses(py::module &sub_module) {
}