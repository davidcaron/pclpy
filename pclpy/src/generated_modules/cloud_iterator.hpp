
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


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
    cls.def("is_valid", &Class::isValid);
        
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
    cls.def("is_valid", &Class::isValid);
        
}

void defineCloudIteratorClasses(py::module &sub_module) {
}