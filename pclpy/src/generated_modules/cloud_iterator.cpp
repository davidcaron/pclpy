
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include <pcl/cloud_iterator.h>



template <typename PointT>
void defineCloudIterator(py::module &m, std::string const & suffix) {
    using Class = pcl::CloudIterator<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointCloud<PointT>>(), "cloud"_a);
    cls.def(py::init<PointCloud<PointT>, std::vector<int>>(), "cloud"_a, "indices"_a);
    cls.def(py::init<PointCloud<PointT>, PointIndices>(), "cloud"_a, "indices"_a);
    cls.def(py::init<PointCloud<PointT>, Correspondences, bool>(), "cloud"_a, "corrs"_a, "source"_a);
    cls.def(py::init<>());
    // Operators not implemented (operator++);
    // Operators not implemented (operator++);
    // Operators not implemented (operator*);
    // Operators not implemented (operator->);
    cls.def("size", &Class::size);
    cls.def("reset", &Class::reset);
    cls.def("isValid", &Class::isValid);
    cls.def("getCurrentPointIndex", &Class::getCurrentPointIndex);
    cls.def("getCurrentIndex", &Class::getCurrentIndex);
        
}

template <typename PointT>
void defineConstCloudIterator(py::module &m, std::string const & suffix) {
    using Class = pcl::ConstCloudIterator<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointCloud<PointT>>(), "cloud"_a);
    cls.def(py::init<PointCloud<PointT>, std::vector<int>>(), "cloud"_a, "indices"_a);
    cls.def(py::init<PointCloud<PointT>, PointIndices>(), "cloud"_a, "indices"_a);
    cls.def(py::init<PointCloud<PointT>, Correspondences, bool>(), "cloud"_a, "corrs"_a, "source"_a);
    cls.def(py::init<>());
    // Operators not implemented (operator++);
    // Operators not implemented (operator++);
    // Operators not implemented (operator*);
    // Operators not implemented (operator->);
    cls.def("size", &Class::size);
    cls.def("reset", &Class::reset);
    cls.def("isValid", &Class::isValid);
    cls.def("getCurrentPointIndex", &Class::getCurrentPointIndex);
    cls.def("getCurrentIndex", &Class::getCurrentIndex);
        
}

void defineCloudIteratorFunctions(py::module &m) {
}

void defineCloudIteratorClasses(py::module &sub_module) {
}