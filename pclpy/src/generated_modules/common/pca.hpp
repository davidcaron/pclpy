
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/pca.h>



template <typename PointT>
void defineCommonPCA(py::module &m, std::string const & suffix) {
    using Class = PCA<PointT>;
    using Base = Class::Base;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::FLAG>(cls, "flag")
        .value("increase", Class::FLAG::increase)
        .value("preserve", Class::FLAG::preserve)
        .export_values();
    cls.def(py::init<bool>(), "basis_only"_a=false);
    cls.def(py::init<pcl::PointCloud<PointT>, bool>(), "X"_a, "basis_only"_a=false);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("project", py::overload_cast<const PointT &, PointT &> (&Class::project));
    cls.def("project", py::overload_cast<const PointCloud &, PointCloud &> (&Class::project));
    cls.def("reconstruct", py::overload_cast<const PointT &, PointT &> (&Class::reconstruct));
    cls.def("reconstruct", py::overload_cast<const PointCloud &, PointCloud &> (&Class::reconstruct));
    // Operators not implemented (operator=);
    cls.def("update", &Class::update);
    cls.def("set_indices", py::overload_cast<const IndicesPtr &> (&Class::setIndices));
    cls.def("set_indices", py::overload_cast<const IndicesConstPtr &> (&Class::setIndices));
    cls.def("set_indices", py::overload_cast<const PointIndicesConstPtr &> (&Class::setIndices));
    cls.def("set_indices", py::overload_cast<size_t, size_t, size_t, size_t> (&Class::setIndices));
        
}

void defineCommonPcaClasses(py::module &sub_module) {
}