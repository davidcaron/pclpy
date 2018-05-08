
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

#include <pcl/common/pca.h>



template <typename PointT>
void defineCommonPCA(py::module &m, std::string const & suffix) {
    using Class = pcl::PCA<PointT>;
    using Base = Class::Base;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::FLAG>(cls, "FLAG")
        .value("increase", Class::FLAG::increase)
        .value("preserve", Class::FLAG::preserve)
        .export_values();
    cls.def(py::init<bool>(), "basis_only"_a=false);
    cls.def(py::init<pcl::PointCloud<PointT>, bool>(), "X"_a, "basis_only"_a=false);
    // Operators not implemented (operator=);
    cls.def("update", &Class::update, "input"_a, "flag"_a=preserve);
    cls.def("project", py::overload_cast<const PointT &, PointT &> (&Class::project), "input"_a, "projection"_a);
    cls.def("project", py::overload_cast<const PointCloud &, PointCloud &> (&Class::project), "input"_a, "projection"_a);
    cls.def("reconstruct", py::overload_cast<const PointT &, PointT &> (&Class::reconstruct), "projection"_a, "input"_a);
    cls.def("reconstruct", py::overload_cast<const PointCloud &, PointCloud &> (&Class::reconstruct), "projection"_a, "input"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setIndices", py::overload_cast<const pcl::IndicesPtr &> (&Class::setIndices), "indices"_a);
    cls.def("setIndices", py::overload_cast<const pcl::IndicesConstPtr &> (&Class::setIndices), "indices"_a);
    cls.def("setIndices", py::overload_cast<const PointIndicesConstPtr &> (&Class::setIndices), "indices"_a);
    cls.def("setIndices", py::overload_cast<size_t, size_t, size_t, size_t> (&Class::setIndices), "row_start"_a, "col_start"_a, "nb_rows"_a, "nb_cols"_a);
    cls.def("getMean", &Class::getMean);
    cls.def("getEigenVectors", &Class::getEigenVectors);
    cls.def("getEigenValues", &Class::getEigenValues);
    cls.def("getCoefficients", &Class::getCoefficients);
        
}

void defineCommonPcaFunctions(py::module &m) {
}

void defineCommonPcaClasses(py::module &sub_module) {
}