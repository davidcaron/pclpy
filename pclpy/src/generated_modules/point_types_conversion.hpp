
#include <pcl/point_types_conversion.h>



void definePointTypesConversionFunctions1(py::module &m) {
    m.def("PointCloudDepthAndRGBtoXYZRGBA", py::overload_cast<const pcl::PointCloud<pcl::Intensity> &, const pcl::PointCloud<pcl::RGB> &, const float &, pcl::PointCloud<pcl::PointXYZRGBA> &> (&pcl::PointCloudDepthAndRGBtoXYZRGBA), "depth"_a, "image"_a, "focal"_a, "out"_a);
    m.def("PointCloudRGBtoI", py::overload_cast<const pcl::PointCloud<pcl::RGB> &, pcl::PointCloud<pcl::Intensity> &> (&pcl::PointCloudRGBtoI), "in"_a, "out"_a);
    m.def("PointCloudRGBtoI", py::overload_cast<const pcl::PointCloud<pcl::RGB> &, pcl::PointCloud<pcl::Intensity8u> &> (&pcl::PointCloudRGBtoI), "in"_a, "out"_a);
    m.def("PointCloudRGBtoI", py::overload_cast<const pcl::PointCloud<pcl::RGB> &, pcl::PointCloud<pcl::Intensity32u> &> (&pcl::PointCloudRGBtoI), "in"_a, "out"_a);
    m.def("PointCloudXYZRGBAtoXYZHSV", py::overload_cast<const pcl::PointCloud<pcl::PointXYZRGBA> &, pcl::PointCloud<pcl::PointXYZHSV> &> (&pcl::PointCloudXYZRGBAtoXYZHSV), "in"_a, "out"_a);
    m.def("PointCloudXYZRGBtoXYZHSV", py::overload_cast<const pcl::PointCloud<pcl::PointXYZRGB> &, pcl::PointCloud<pcl::PointXYZHSV> &> (&pcl::PointCloudXYZRGBtoXYZHSV), "in"_a, "out"_a);
    m.def("PointCloudXYZRGBtoXYZI", py::overload_cast<const pcl::PointCloud<pcl::PointXYZRGB> &, pcl::PointCloud<pcl::PointXYZI> &> (&pcl::PointCloudXYZRGBtoXYZI), "in"_a, "out"_a);
    m.def("PointRGBtoI", py::overload_cast<const pcl::RGB &, pcl::Intensity &> (&pcl::PointRGBtoI), "in"_a, "out"_a);
    m.def("PointRGBtoI", py::overload_cast<const pcl::RGB &, pcl::Intensity8u &> (&pcl::PointRGBtoI), "in"_a, "out"_a);
    m.def("PointRGBtoI", py::overload_cast<const pcl::RGB &, pcl::Intensity32u &> (&pcl::PointRGBtoI), "in"_a, "out"_a);
    m.def("PointXYZHSVtoXYZRGB", py::overload_cast<const pcl::PointXYZHSV &, pcl::PointXYZRGB &> (&pcl::PointXYZHSVtoXYZRGB), "in"_a, "out"_a);
    m.def("PointXYZRGBAtoXYZHSV", py::overload_cast<const pcl::PointXYZRGBA &, pcl::PointXYZHSV &> (&pcl::PointXYZRGBAtoXYZHSV), "in"_a, "out"_a);
    m.def("PointXYZRGBtoXYZHSV", py::overload_cast<const pcl::PointXYZRGB &, pcl::PointXYZHSV &> (&pcl::PointXYZRGBtoXYZHSV), "in"_a, "out"_a);
    m.def("PointXYZRGBtoXYZI", py::overload_cast<const pcl::PointXYZRGB &, pcl::PointXYZI &> (&pcl::PointXYZRGBtoXYZI), "in"_a, "out"_a);
}

void definePointTypesConversionFunctions(py::module &m) {
    definePointTypesConversionFunctions1(m);
}

void definePointTypesConversionClasses(py::module &sub_module) {
    definePointTypesConversionFunctions(sub_module);
}