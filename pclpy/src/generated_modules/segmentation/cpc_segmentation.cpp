
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/segmentation/cpc_segmentation.h>



template <typename PointT>
void defineSegmentationCPCSegmentation(py::module &m, std::string const & suffix) {
    using Class = pcl::CPCSegmentation<PointT>;
    py::class_<Class, pcl::LCCPSegmentation<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("segment", py::overload_cast<> (&Class::segment));
    cls.def("setCutting", &Class::setCutting, "max_cuts"_a=20, "cutting_min_segments"_a=0, "cutting_min_score"_a=0.16, "locally_constrained"_a=true, "directed_cutting"_a=true, "clean_cutting"_a=false);
    cls.def("setRANSACIterations", &Class::setRANSACIterations, "ransac_iterations"_a);
        
}

void defineSegmentationCpcSegmentationFunctions(py::module &m) {
}

void defineSegmentationCpcSegmentationClasses(py::module &sub_module) {
    py::module sub_module_CPCSegmentation = sub_module.def_submodule("CPCSegmentation", "Submodule CPCSegmentation");
    defineSegmentationCPCSegmentation<pcl::PointXYZ>(sub_module_CPCSegmentation, "PointXYZ");
    defineSegmentationCPCSegmentation<pcl::PointXYZRGB>(sub_module_CPCSegmentation, "PointXYZRGB");
    defineSegmentationCPCSegmentation<pcl::PointXYZRGBA>(sub_module_CPCSegmentation, "PointXYZRGBA");
    defineSegmentationCPCSegmentation<pcl::PointXYZRGBNormal>(sub_module_CPCSegmentation, "PointXYZRGBNormal");
}