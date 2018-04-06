
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/extract_polygonal_prism_data.h>



template <typename PointT>
void defineSegmentationExtractPolygonalPrismData(py::module &m, std::string const & suffix) {
    using Class = ExtractPolygonalPrismData<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("input_planar_hull", &Class::getInputPlanarHull, &Class::setInputPlanarHull);
    cls.def_property("height_limits", &Class::getHeightLimits, &Class::setHeightLimits);
    cls.def_property("view_point", &Class::getViewPoint, &Class::setViewPoint);
    cls.def("segment", py::overload_cast<PointIndices &> (&Class::segment));
        
}

void defineSegmentationExtractPolygonalPrismDataClasses(py::module &sub_module) {
    py::module sub_module_ExtractPolygonalPrismData = sub_module.def_submodule("ExtractPolygonalPrismData", "Submodule ExtractPolygonalPrismData");
    defineSegmentationExtractPolygonalPrismData<PointXYZ>(sub_module_ExtractPolygonalPrismData, "PointXYZ");
    defineSegmentationExtractPolygonalPrismData<PointXYZI>(sub_module_ExtractPolygonalPrismData, "PointXYZI");
    defineSegmentationExtractPolygonalPrismData<PointXYZRGB>(sub_module_ExtractPolygonalPrismData, "PointXYZRGB");
    defineSegmentationExtractPolygonalPrismData<PointXYZRGBA>(sub_module_ExtractPolygonalPrismData, "PointXYZRGBA");
}