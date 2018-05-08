
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include <pcl/point_cloud.h>



template <typename PointOutT>
void defineNdCopyEigenPointFunctor(py::module &m, std::string const & suffix) {
    using Class = pcl::NdCopyEigenPointFunctor<PointOutT>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<Eigen::VectorXf, PointOutT>(), "p1"_a, "p2"_a);
    // Operators not implemented (operator());
        
}

template <typename PointInT>
void defineNdCopyPointEigenFunctor(py::module &m, std::string const & suffix) {
    using Class = pcl::NdCopyPointEigenFunctor<PointInT>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointInT, Eigen::VectorXf>(), "p1"_a, "p2"_a);
    // Operators not implemented (operator());
        
}

template <typename PointT>
void definePointCloud(py::module &m, std::string const & suffix) {
    using Class = pcl::PointCloud<PointT>;
    using PointType = Class::PointType;
    using VectorType = Class::VectorType;
    using CloudVectorType = Class::CloudVectorType;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using value_type = Class::value_type;
    using reference = Class::reference;
    using const_reference = Class::const_reference;
    using difference_type = Class::difference_type;
    using size_type = Class::size_type;
    using iterator = Class::iterator;
    using const_iterator = Class::const_iterator;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<pcl::PointCloud<PointT>, std::vector<int>>(), "pc"_a, "indices"_a);
    cls.def(py::init<uint32_t, uint32_t, PointT>(), "width_"_a, "height_"_a, "value_"_a=PointT());
    cls.def_readwrite("header", &Class::header);
    cls.def_readwrite("points", &Class::points);
    cls.def_readwrite("width", &Class::width);
    cls.def_readwrite("height", &Class::height);
    cls.def_readwrite("is_dense", &Class::is_dense);
    cls.def_readwrite("sensor_origin_", &Class::sensor_origin_);
    cls.def_readwrite("sensor_orientation_", &Class::sensor_orientation_);
    // Operators not implemented (operator+=);
    // Operators not implemented (operator+);
    cls.def("at", py::overload_cast<int, int> (&Class::at, py::const_), "column"_a, "row"_a);
    cls.def("at", py::overload_cast<int, int> (&Class::at), "column"_a, "row"_a);
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("isOrganized", &Class::isOrganized);
    cls.def("begin", py::overload_cast<> (&Class::begin));
    cls.def("end", py::overload_cast<> (&Class::end));
    cls.def("begin", py::overload_cast<> (&Class::begin, py::const_));
    cls.def("end", py::overload_cast<> (&Class::end, py::const_));
    cls.def("size", &Class::size);
    cls.def("reserve", &Class::reserve, "n"_a);
    cls.def("empty", &Class::empty);
    cls.def("resize", &Class::resize, "n"_a);
    // Operators not implemented (operator[]);
    // Operators not implemented (operator[]);
    cls.def("at", py::overload_cast<size_t> (&Class::at, py::const_), "n"_a);
    cls.def("at", py::overload_cast<size_t> (&Class::at), "n"_a);
    cls.def("front", py::overload_cast<> (&Class::front, py::const_));
    cls.def("front", py::overload_cast<> (&Class::front));
    cls.def("back", py::overload_cast<> (&Class::back, py::const_));
    cls.def("back", py::overload_cast<> (&Class::back));
    cls.def("push_back", &Class::push_back, "pt"_a);
    cls.def("erase", py::overload_cast<iterator> (&Class::erase), "position"_a);
    cls.def("erase", py::overload_cast<iterator, iterator> (&Class::erase), "first"_a, "last"_a);
    cls.def("swap", &Class::swap, "rhs"_a);
    cls.def("clear", &Class::clear);
    cls.def("makeShared", &Class::makeShared);
    cls.def("getMatrixXfMap", py::overload_cast<int, int, int> (&Class::getMatrixXfMap), "dim"_a, "stride"_a, "offset"_a);
    cls.def("getMatrixXfMap", py::overload_cast<int, int, int> (&Class::getMatrixXfMap, py::const_), "dim"_a, "stride"_a, "offset"_a);
    cls.def("getMatrixXfMap", py::overload_cast<> (&Class::getMatrixXfMap));
    cls.def("getMatrixXfMap", py::overload_cast<> (&Class::getMatrixXfMap, py::const_));
        
}

void definePointCloudFunctions(py::module &m) {
}

void definePointCloudClasses(py::module &sub_module) {
    py::module sub_module_PointCloud = sub_module.def_submodule("PointCloud", "Submodule PointCloud");
    definePointCloud<pcl::Axis>(sub_module_PointCloud, "Axis");
    definePointCloud<pcl::BRISKSignature512>(sub_module_PointCloud, "BRISKSignature512");
    definePointCloud<pcl::Boundary>(sub_module_PointCloud, "Boundary");
    definePointCloud<pcl::CPPFSignature>(sub_module_PointCloud, "CPPFSignature");
    definePointCloud<pcl::ESFSignature640>(sub_module_PointCloud, "ESFSignature640");
    definePointCloud<pcl::FPFHSignature33>(sub_module_PointCloud, "FPFHSignature33");
    definePointCloud<pcl::GRSDSignature21>(sub_module_PointCloud, "GRSDSignature21");
    definePointCloud<pcl::IntensityGradient>(sub_module_PointCloud, "IntensityGradient");
    definePointCloud<pcl::InterestPoint>(sub_module_PointCloud, "InterestPoint");
    definePointCloud<pcl::Label>(sub_module_PointCloud, "Label");
    definePointCloud<pcl::MomentInvariants>(sub_module_PointCloud, "MomentInvariants");
    definePointCloud<pcl::Narf36>(sub_module_PointCloud, "Narf36");
    definePointCloud<pcl::Normal>(sub_module_PointCloud, "Normal");
    definePointCloud<pcl::NormalBasedSignature12>(sub_module_PointCloud, "NormalBasedSignature12");
    definePointCloud<pcl::PFHRGBSignature250>(sub_module_PointCloud, "PFHRGBSignature250");
    definePointCloud<pcl::PFHSignature125>(sub_module_PointCloud, "PFHSignature125");
    definePointCloud<pcl::PPFRGBSignature>(sub_module_PointCloud, "PPFRGBSignature");
    definePointCloud<pcl::PPFSignature>(sub_module_PointCloud, "PPFSignature");
    definePointCloud<pcl::PointDEM>(sub_module_PointCloud, "PointDEM");
    definePointCloud<pcl::PointNormal>(sub_module_PointCloud, "PointNormal");
    definePointCloud<pcl::PointSurfel>(sub_module_PointCloud, "PointSurfel");
    definePointCloud<pcl::PointUV>(sub_module_PointCloud, "PointUV");
    definePointCloud<pcl::PointWithRange>(sub_module_PointCloud, "PointWithRange");
    definePointCloud<pcl::PointWithScale>(sub_module_PointCloud, "PointWithScale");
    definePointCloud<pcl::PointWithViewpoint>(sub_module_PointCloud, "PointWithViewpoint");
    definePointCloud<pcl::PointXY>(sub_module_PointCloud, "PointXY");
    definePointCloud<pcl::PointXYZ>(sub_module_PointCloud, "PointXYZ");
    definePointCloud<pcl::PointXYZHSV>(sub_module_PointCloud, "PointXYZHSV");
    definePointCloud<pcl::PointXYZI>(sub_module_PointCloud, "PointXYZI");
    definePointCloud<pcl::PointXYZINormal>(sub_module_PointCloud, "PointXYZINormal");
    definePointCloud<pcl::PointXYZL>(sub_module_PointCloud, "PointXYZL");
    definePointCloud<pcl::PointXYZLNormal>(sub_module_PointCloud, "PointXYZLNormal");
    definePointCloud<pcl::PointXYZRGB>(sub_module_PointCloud, "PointXYZRGB");
    definePointCloud<pcl::PointXYZRGBA>(sub_module_PointCloud, "PointXYZRGBA");
    definePointCloud<pcl::PointXYZRGBL>(sub_module_PointCloud, "PointXYZRGBL");
    definePointCloud<pcl::PointXYZRGBNormal>(sub_module_PointCloud, "PointXYZRGBNormal");
    definePointCloud<pcl::PrincipalCurvatures>(sub_module_PointCloud, "PrincipalCurvatures");
    definePointCloud<pcl::PrincipalRadiiRSD>(sub_module_PointCloud, "PrincipalRadiiRSD");
    definePointCloud<pcl::ReferenceFrame>(sub_module_PointCloud, "ReferenceFrame");
    definePointCloud<pcl::SHOT1344>(sub_module_PointCloud, "SHOT1344");
    definePointCloud<pcl::SHOT352>(sub_module_PointCloud, "SHOT352");
    definePointCloud<pcl::ShapeContext1980>(sub_module_PointCloud, "ShapeContext1980");
    definePointCloud<pcl::UniqueShapeContext1960>(sub_module_PointCloud, "UniqueShapeContext1960");
    definePointCloud<pcl::VFHSignature308>(sub_module_PointCloud, "VFHSignature308");
    definePointCloudFunctions(sub_module);
}