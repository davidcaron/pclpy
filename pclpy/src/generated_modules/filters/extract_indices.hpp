
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/extract_indices.h>



template<typename PointT>
void defineFiltersExtractIndices(py::module &m, std::string const & suffix) {
    using Class = ExtractIndices<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def("filter_directly", &Class::filterDirectly);
        
}



void defineFiltersExtractIndicesClasses(py::module &sub_module) {
    py::module sub_module_ExtractIndices = sub_module.def_submodule("ExtractIndices", "Submodule ExtractIndices");
    defineFiltersExtractIndices<Normal>(sub_module_ExtractIndices, "Normal");
    defineFiltersExtractIndices<PointXYZ>(sub_module_ExtractIndices, "PointXYZ");
    defineFiltersExtractIndices<PointXYZI>(sub_module_ExtractIndices, "PointXYZI");
    defineFiltersExtractIndices<PointXYZRGB>(sub_module_ExtractIndices, "PointXYZRGB");
    defineFiltersExtractIndices<PointXYZRGBA>(sub_module_ExtractIndices, "PointXYZRGBA");
    defineFiltersExtractIndices<PointXYZRGBNormal>(sub_module_ExtractIndices, "PointXYZRGBNormal");
}