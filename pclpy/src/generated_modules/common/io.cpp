
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

#include <pcl/common/io.h>



void defineCommonIoFunctions1(py::module &m) {
    m.def("concatenatePointCloud", py::overload_cast<const pcl::PCLPointCloud2 &, const pcl::PCLPointCloud2 &, pcl::PCLPointCloud2 &> (&pcl::concatenatePointCloud), "cloud1"_a, "cloud2"_a, "cloud_out"_a);
    m.def("getEigenAsPointCloud", py::overload_cast<Eigen::MatrixXf &, pcl::PCLPointCloud2 &> (&pcl::getEigenAsPointCloud), "in"_a, "out"_a);
    m.def("getFieldSize", py::overload_cast<const int> (&pcl::getFieldSize), "datatype"_a);
    m.def("getFieldType", py::overload_cast<const int, char> (&pcl::getFieldType), "size"_a, "type"_a);
    m.def("getFieldType", py::overload_cast<const int> (&pcl::getFieldType), "type"_a);
    m.def("getPointCloudAsEigen", py::overload_cast<const pcl::PCLPointCloud2 &, Eigen::MatrixXf &> (&pcl::getPointCloudAsEigen), "in"_a, "out"_a);
    m.def("interpolatePointIndex", py::overload_cast<int, int, pcl::InterpolationType> (&pcl::interpolatePointIndex), "p"_a, "length"_a, "type"_a);
}

template <typename PointT>
void defineCommonIoFunctions2(py::module &m) {
    m.def("getFields", py::overload_cast<const pcl::PointCloud<PointT> &, std::vector<pcl::PCLPointField> &> (&pcl::getFields<PointT>), "cloud"_a, "fields"_a);
    m.def("getFields", py::overload_cast<std::vector<pcl::PCLPointField> &> (&pcl::getFields<PointT>), "fields"_a);
}

void defineCommonIoFunctions(py::module &m) {
    defineCommonIoFunctions1(m);
    defineCommonIoFunctions2<pcl::PointXYZ>(m);
    defineCommonIoFunctions2<pcl::PointXYZI>(m);
    defineCommonIoFunctions2<pcl::PointXYZL>(m);
    defineCommonIoFunctions2<pcl::Label>(m);
    defineCommonIoFunctions2<pcl::PointXYZRGBA>(m);
    defineCommonIoFunctions2<pcl::PointXYZRGB>(m);
    defineCommonIoFunctions2<pcl::PointXYZRGBL>(m);
    defineCommonIoFunctions2<pcl::PointXYZHSV>(m);
    defineCommonIoFunctions2<pcl::PointXY>(m);
    defineCommonIoFunctions2<pcl::InterestPoint>(m);
    defineCommonIoFunctions2<pcl::Axis>(m);
    defineCommonIoFunctions2<pcl::Normal>(m);
    defineCommonIoFunctions2<pcl::PointNormal>(m);
    defineCommonIoFunctions2<pcl::PointXYZRGBNormal>(m);
    defineCommonIoFunctions2<pcl::PointXYZINormal>(m);
    defineCommonIoFunctions2<pcl::PointXYZLNormal>(m);
    defineCommonIoFunctions2<pcl::PointWithRange>(m);
    defineCommonIoFunctions2<pcl::PointWithViewpoint>(m);
    defineCommonIoFunctions2<pcl::MomentInvariants>(m);
    defineCommonIoFunctions2<pcl::PrincipalRadiiRSD>(m);
    defineCommonIoFunctions2<pcl::Boundary>(m);
    defineCommonIoFunctions2<pcl::PrincipalCurvatures>(m);
    defineCommonIoFunctions2<pcl::PFHSignature125>(m);
    defineCommonIoFunctions2<pcl::PFHRGBSignature250>(m);
    defineCommonIoFunctions2<pcl::PPFSignature>(m);
    defineCommonIoFunctions2<pcl::CPPFSignature>(m);
    defineCommonIoFunctions2<pcl::PPFRGBSignature>(m);
    defineCommonIoFunctions2<pcl::NormalBasedSignature12>(m);
    defineCommonIoFunctions2<pcl::FPFHSignature33>(m);
    defineCommonIoFunctions2<pcl::VFHSignature308>(m);
    defineCommonIoFunctions2<pcl::GRSDSignature21>(m);
    defineCommonIoFunctions2<pcl::ESFSignature640>(m);
    defineCommonIoFunctions2<pcl::BRISKSignature512>(m);
    defineCommonIoFunctions2<pcl::Narf36>(m);
    defineCommonIoFunctions2<pcl::IntensityGradient>(m);
    defineCommonIoFunctions2<pcl::PointWithScale>(m);
    defineCommonIoFunctions2<pcl::PointSurfel>(m);
    defineCommonIoFunctions2<pcl::ShapeContext1980>(m);
    defineCommonIoFunctions2<pcl::UniqueShapeContext1960>(m);
    defineCommonIoFunctions2<pcl::SHOT352>(m);
    defineCommonIoFunctions2<pcl::SHOT1344>(m);
    defineCommonIoFunctions2<pcl::PointUV>(m);
    defineCommonIoFunctions2<pcl::ReferenceFrame>(m);
    defineCommonIoFunctions2<pcl::PointDEM>(m);
}

void defineCommonIoClasses(py::module &sub_module) {
    defineCommonIoFunctions(sub_module);
    py::enum_<pcl::InterpolationType>(sub_module, "InterpolationType")
        .value("BORDER_CONSTANT", pcl::InterpolationType::BORDER_CONSTANT)
        .value("BORDER_REPLICATE", pcl::InterpolationType::BORDER_REPLICATE)
        .value("BORDER_REFLECT", pcl::InterpolationType::BORDER_REFLECT)
        .value("BORDER_WRAP", pcl::InterpolationType::BORDER_WRAP)
        .value("BORDER_REFLECT_101", pcl::InterpolationType::BORDER_REFLECT_101)
        .value("BORDER_TRANSPARENT", pcl::InterpolationType::BORDER_TRANSPARENT)
        .value("BORDER_DEFAULT", pcl::InterpolationType::BORDER_DEFAULT)
        .export_values();
}