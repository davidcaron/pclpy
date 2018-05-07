
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/point_traits.h>



template <typename PointInT, typename OutT>
void defineCopyIfFieldExists(py::module &m, std::string const & suffix) {
    using Class = pcl::CopyIfFieldExists<PointInT, OutT>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointInT, std::string, bool, OutT>(), "pt"_a, "field"_a, "exists"_a, "value"_a);
    cls.def(py::init<PointInT, std::string, OutT>(), "pt"_a, "field"_a, "value"_a);
    // Operators not implemented (operator());
        
}

template<typename PointT, typename Tag>
void defineFieldMatches(py::module &m, std::string const & suffix) {
    using Class = pcl::FieldMatches<PointT, Tag>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    // Operators not implemented (operator());
        
}

template <typename PointOutT, typename InT>
void defineSetIfFieldExists(py::module &m, std::string const & suffix) {
    using Class = pcl::SetIfFieldExists<PointOutT, InT>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointOutT, std::string, InT>(), "pt"_a, "field"_a, "value"_a);
    // Operators not implemented (operator());
        
}

void definePointTraitsFunctions(py::module &m) {
}

void definePointTraitsClasses(py::module &sub_module) {
    definePointTraitsFunctions(sub_module);
}