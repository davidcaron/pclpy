
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/bilateral.h>



template<typename PointT>
void defineFiltersBilateralFilter(py::module &m, std::string const & suffix) {
    using Class = BilateralFilter<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("half_size", &Class::getHalfSize, &Class::setHalfSize);
    cls.def_property("std_dev", &Class::getStdDev, &Class::setStdDev);
    cls.def("set_search_method", &Class::setSearchMethod);
    cls.def("apply_filter", py::overload_cast<Filter<PointT>::PointCloud &> (&Class::applyFilter));
    cls.def("compute_point_weight", &Class::computePointWeight);
        
}

void defineFiltersBilateralClasses(py::module &sub_module) {
    py::module sub_module_BilateralFilter = sub_module.def_submodule("BilateralFilter", "Submodule BilateralFilter");
    defineFiltersBilateralFilter<PointXYZI>(sub_module_BilateralFilter, "PointXYZI");
    defineFiltersBilateralFilter<PointXYZINormal>(sub_module_BilateralFilter, "PointXYZINormal");
}