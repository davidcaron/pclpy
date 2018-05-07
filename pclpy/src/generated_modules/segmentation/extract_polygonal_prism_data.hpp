
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/extract_polygonal_prism_data.h>



template <typename PointT>
void defineSegmentationExtractPolygonalPrismData(py::module &m, std::string const & suffix) {
    using Class = pcl::ExtractPolygonalPrismData<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("segment", py::overload_cast<pcl::PointIndices &> (&Class::segment), "output"_a);
    cls.def("setInputPlanarHull", &Class::setInputPlanarHull, "hull"_a);
    cls.def("setHeightLimits", &Class::setHeightLimits, "height_min"_a, "height_max"_a);
    cls.def("setViewPoint", &Class::setViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
    cls.def("getInputPlanarHull", &Class::getInputPlanarHull);
    cls.def("getHeightLimits", &Class::getHeightLimits, "height_min"_a, "height_max"_a);
    cls.def("getViewPoint", &Class::getViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
        
}

void defineSegmentationExtractPolygonalPrismDataFunctions(py::module &m) {
}

void defineSegmentationExtractPolygonalPrismDataClasses(py::module &sub_module) {
    py::module sub_module_ExtractPolygonalPrismData = sub_module.def_submodule("ExtractPolygonalPrismData", "Submodule ExtractPolygonalPrismData");
    defineSegmentationExtractPolygonalPrismData<pcl::PointXYZ>(sub_module_ExtractPolygonalPrismData, "PointXYZ");
    defineSegmentationExtractPolygonalPrismData<pcl::PointXYZI>(sub_module_ExtractPolygonalPrismData, "PointXYZI");
    defineSegmentationExtractPolygonalPrismData<pcl::PointXYZRGB>(sub_module_ExtractPolygonalPrismData, "PointXYZRGB");
    defineSegmentationExtractPolygonalPrismData<pcl::PointXYZRGBA>(sub_module_ExtractPolygonalPrismData, "PointXYZRGBA");
    defineSegmentationExtractPolygonalPrismDataFunctions(sub_module);
}