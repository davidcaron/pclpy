
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/cpc_segmentation.h>



template <typename PointT>
void defineSegmentationCPCSegmentation(py::module &m, std::string const & suffix) {
    using Class = CPCSegmentation<PointT>;
    py::class_<Class, LCCPSegmentation<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_cutting", &Class::setCutting);
    cls.def("set_ransac_iterations", &Class::setRANSACIterations);
    cls.def("segment", py::overload_cast<> (&Class::segment));
        
}

void defineSegmentationCpcSegmentationClasses(py::module &sub_module) {
    py::module sub_module_CPCSegmentation = sub_module.def_submodule("CPCSegmentation", "Submodule CPCSegmentation");
    defineSegmentationCPCSegmentation<PointXYZ>(sub_module_CPCSegmentation, "PointXYZ");
    defineSegmentationCPCSegmentation<PointXYZRGB>(sub_module_CPCSegmentation, "PointXYZRGB");
    defineSegmentationCPCSegmentation<PointXYZRGBA>(sub_module_CPCSegmentation, "PointXYZRGBA");
    defineSegmentationCPCSegmentation<PointXYZRGBNormal>(sub_module_CPCSegmentation, "PointXYZRGBNormal");
}