
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

#include <pcl/filters/fast_bilateral_omp.h>



template<typename PointT>
void defineFiltersFastBilateralFilterOMP(py::module &m, std::string const & suffix) {
    using Class = pcl::FastBilateralFilterOMP<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FastBilateralFilter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "nr_threads"_a=0);
    cls.def("applyFilter", py::overload_cast<pcl::Filter<PointT>::PointCloud &> (&Class::applyFilter), "output"_a);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineFiltersFastBilateralOmpFunctions(py::module &m) {
}

void defineFiltersFastBilateralOmpClasses(py::module &sub_module) {
    py::module sub_module_FastBilateralFilterOMP = sub_module.def_submodule("FastBilateralFilterOMP", "Submodule FastBilateralFilterOMP");
    defineFiltersFastBilateralFilterOMP<pcl::PointXYZ>(sub_module_FastBilateralFilterOMP, "PointXYZ");
    defineFiltersFastBilateralFilterOMP<pcl::PointXYZRGB>(sub_module_FastBilateralFilterOMP, "PointXYZRGB");
    defineFiltersFastBilateralFilterOMP<pcl::PointXYZRGBA>(sub_module_FastBilateralFilterOMP, "PointXYZRGBA");
}