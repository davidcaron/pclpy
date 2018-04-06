
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/point_representation.h>



template <typename PointT>
void definePointRepresentation(py::module &m, std::string const & suffix) {
    using Class = PointRepresentation<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("set_rescale_values", &Class::setRescaleValues);
    cls.def("copy_to_float_array", &Class::copyToFloatArray);
    cls.def("is_trivial", &Class::isTrivial);
    cls.def("is_valid", &Class::isValid);
        
}

template <typename PointDefault>
void defineCustomPointRepresentation(py::module &m, std::string const & suffix) {
    using Class = CustomPointRepresentation<PointDefault>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PointRepresentation<PointDefault>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<int, int>(), "max_dim"_a=3, "start_dim"_a=0);
    cls.def("make_shared", &Class::makeShared);
    cls.def("copy_to_float_array", &Class::copyToFloatArray);
        
}

template <typename PointDefault>
void defineDefaultFeatureRepresentation(py::module &m, std::string const & suffix) {
    using Class = DefaultFeatureRepresentation<PointDefault>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using FieldList = Class::FieldList;
    py::class_<Class, PointRepresentation<PointDefault>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("make_shared", &Class::makeShared);
    cls.def("copy_to_float_array", &Class::copyToFloatArray);
        
}

template <typename PointDefault>
void defineDefaultPointRepresentation(py::module &m, std::string const & suffix) {
    using Class = DefaultPointRepresentation<PointDefault>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PointRepresentation<PointDefault>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("make_shared", &Class::makeShared);
    cls.def("copy_to_float_array", &Class::copyToFloatArray);
        
}

void definePointRepresentationClasses(py::module &sub_module) {
}