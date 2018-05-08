
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

#include <pcl/filters/bilateral.h>



template<typename PointT>
void defineFiltersBilateralFilter(py::module &m, std::string const & suffix) {
    using Class = pcl::BilateralFilter<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("applyFilter", py::overload_cast<pcl::Filter<PointT>::PointCloud &> (&Class::applyFilter), "output"_a);
    cls.def("computePointWeight", &Class::computePointWeight, "pid"_a, "indices"_a, "distances"_a);
    cls.def("setHalfSize", &Class::setHalfSize, "sigma_s"_a);
    cls.def("setStdDev", &Class::setStdDev, "sigma_r"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("getHalfSize", &Class::getHalfSize);
    cls.def("getStdDev", &Class::getStdDev);
        
}

void defineFiltersBilateralFunctions(py::module &m) {
}

void defineFiltersBilateralClasses(py::module &sub_module) {
    py::module sub_module_BilateralFilter = sub_module.def_submodule("BilateralFilter", "Submodule BilateralFilter");
    defineFiltersBilateralFilter<pcl::PointXYZI>(sub_module_BilateralFilter, "PointXYZI");
    defineFiltersBilateralFilter<pcl::PointXYZINormal>(sub_module_BilateralFilter, "PointXYZINormal");
}