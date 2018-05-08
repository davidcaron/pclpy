
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

#include <pcl/filters/extract_indices.h>



template<typename PointT>
void defineFiltersExtractIndices(py::module &m, std::string const & suffix) {
    using Class = pcl::ExtractIndices<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def("filterDirectly", &Class::filterDirectly, "cloud"_a);
        
}

void defineFiltersExtractIndicesFunctions(py::module &m) {
}

void defineFiltersExtractIndicesClasses(py::module &sub_module) {
    py::module sub_module_ExtractIndices = sub_module.def_submodule("ExtractIndices", "Submodule ExtractIndices");
    defineFiltersExtractIndices<pcl::Normal>(sub_module_ExtractIndices, "Normal");
    defineFiltersExtractIndices<pcl::PointXYZ>(sub_module_ExtractIndices, "PointXYZ");
    defineFiltersExtractIndices<pcl::PointXYZI>(sub_module_ExtractIndices, "PointXYZI");
    defineFiltersExtractIndices<pcl::PointXYZRGB>(sub_module_ExtractIndices, "PointXYZRGB");
    defineFiltersExtractIndices<pcl::PointXYZRGBA>(sub_module_ExtractIndices, "PointXYZRGBA");
    defineFiltersExtractIndices<pcl::PointXYZRGBNormal>(sub_module_ExtractIndices, "PointXYZRGBNormal");
}