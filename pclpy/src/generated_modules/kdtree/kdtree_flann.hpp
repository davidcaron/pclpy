
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/kdtree/kdtree_flann.h>



template <typename PointT, typename Dist = ::flann::L2_Simple<float> >
void defineKdtreeKdTreeFLANN(py::module &m, std::string const & suffix) {
    using Class = pcl::KdTreeFLANN<PointT, Dist>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using FLANNIndex = Class::FLANNIndex;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::KdTree<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "sorted"_a=true);
    // Operators not implemented (operator=);
    cls.def("makeShared", &Class::makeShared);
    cls.def("nearestKSearch", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "point"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("radiusSearch", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "point"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("setEpsilon", &Class::setEpsilon, "eps"_a);
    cls.def("setSortedResults", &Class::setSortedResults, "sorted"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a, "indices"_a=pcl::IndicesConstPtr());
        
}

void defineKdtreeKdtreeFlannFunctions(py::module &m) {
}

void defineKdtreeKdtreeFlannClasses(py::module &sub_module) {
    py::module sub_module_KdTreeFLANN = sub_module.def_submodule("KdTreeFLANN", "Submodule KdTreeFLANN");
    defineKdtreeKdTreeFLANN<pcl::Axis>(sub_module_KdTreeFLANN, "Axis");
    defineKdtreeKdTreeFLANN<pcl::BRISKSignature512>(sub_module_KdTreeFLANN, "BRISKSignature512");
    defineKdtreeKdTreeFLANN<pcl::Boundary>(sub_module_KdTreeFLANN, "Boundary");
    defineKdtreeKdTreeFLANN<pcl::CPPFSignature>(sub_module_KdTreeFLANN, "CPPFSignature");
    defineKdtreeKdTreeFLANN<pcl::ESFSignature640>(sub_module_KdTreeFLANN, "ESFSignature640");
    defineKdtreeKdTreeFLANN<pcl::FPFHSignature33>(sub_module_KdTreeFLANN, "FPFHSignature33");
    defineKdtreeKdTreeFLANN<pcl::GRSDSignature21>(sub_module_KdTreeFLANN, "GRSDSignature21");
    defineKdtreeKdTreeFLANN<pcl::IntensityGradient>(sub_module_KdTreeFLANN, "IntensityGradient");
    defineKdtreeKdTreeFLANN<pcl::InterestPoint>(sub_module_KdTreeFLANN, "InterestPoint");
    defineKdtreeKdTreeFLANN<pcl::Label>(sub_module_KdTreeFLANN, "Label");
    defineKdtreeKdTreeFLANN<pcl::MomentInvariants>(sub_module_KdTreeFLANN, "MomentInvariants");
    defineKdtreeKdTreeFLANN<pcl::Narf36>(sub_module_KdTreeFLANN, "Narf36");
    defineKdtreeKdTreeFLANN<pcl::Normal>(sub_module_KdTreeFLANN, "Normal");
    defineKdtreeKdTreeFLANN<pcl::NormalBasedSignature12>(sub_module_KdTreeFLANN, "NormalBasedSignature12");
    defineKdtreeKdTreeFLANN<pcl::PFHRGBSignature250>(sub_module_KdTreeFLANN, "PFHRGBSignature250");
    defineKdtreeKdTreeFLANN<pcl::PFHSignature125>(sub_module_KdTreeFLANN, "PFHSignature125");
    defineKdtreeKdTreeFLANN<pcl::PPFRGBSignature>(sub_module_KdTreeFLANN, "PPFRGBSignature");
    defineKdtreeKdTreeFLANN<pcl::PPFSignature>(sub_module_KdTreeFLANN, "PPFSignature");
    defineKdtreeKdTreeFLANN<pcl::PointDEM>(sub_module_KdTreeFLANN, "PointDEM");
    defineKdtreeKdTreeFLANN<pcl::PointNormal>(sub_module_KdTreeFLANN, "PointNormal");
    defineKdtreeKdTreeFLANN<pcl::PointSurfel>(sub_module_KdTreeFLANN, "PointSurfel");
    defineKdtreeKdTreeFLANN<pcl::PointUV>(sub_module_KdTreeFLANN, "PointUV");
    defineKdtreeKdTreeFLANN<pcl::PointWithRange>(sub_module_KdTreeFLANN, "PointWithRange");
    defineKdtreeKdTreeFLANN<pcl::PointWithScale>(sub_module_KdTreeFLANN, "PointWithScale");
    defineKdtreeKdTreeFLANN<pcl::PointWithViewpoint>(sub_module_KdTreeFLANN, "PointWithViewpoint");
    defineKdtreeKdTreeFLANN<pcl::PointXY>(sub_module_KdTreeFLANN, "PointXY");
    defineKdtreeKdTreeFLANN<pcl::PointXYZ>(sub_module_KdTreeFLANN, "PointXYZ");
    defineKdtreeKdTreeFLANN<pcl::PointXYZHSV>(sub_module_KdTreeFLANN, "PointXYZHSV");
    defineKdtreeKdTreeFLANN<pcl::PointXYZI>(sub_module_KdTreeFLANN, "PointXYZI");
    defineKdtreeKdTreeFLANN<pcl::PointXYZINormal>(sub_module_KdTreeFLANN, "PointXYZINormal");
    defineKdtreeKdTreeFLANN<pcl::PointXYZL>(sub_module_KdTreeFLANN, "PointXYZL");
    defineKdtreeKdTreeFLANN<pcl::PointXYZLNormal>(sub_module_KdTreeFLANN, "PointXYZLNormal");
    defineKdtreeKdTreeFLANN<pcl::PointXYZRGB>(sub_module_KdTreeFLANN, "PointXYZRGB");
    defineKdtreeKdTreeFLANN<pcl::PointXYZRGBA>(sub_module_KdTreeFLANN, "PointXYZRGBA");
    defineKdtreeKdTreeFLANN<pcl::PointXYZRGBL>(sub_module_KdTreeFLANN, "PointXYZRGBL");
    defineKdtreeKdTreeFLANN<pcl::PointXYZRGBNormal>(sub_module_KdTreeFLANN, "PointXYZRGBNormal");
    defineKdtreeKdTreeFLANN<pcl::PrincipalCurvatures>(sub_module_KdTreeFLANN, "PrincipalCurvatures");
    defineKdtreeKdTreeFLANN<pcl::PrincipalRadiiRSD>(sub_module_KdTreeFLANN, "PrincipalRadiiRSD");
    defineKdtreeKdTreeFLANN<pcl::ReferenceFrame>(sub_module_KdTreeFLANN, "ReferenceFrame");
    defineKdtreeKdTreeFLANN<pcl::SHOT1344>(sub_module_KdTreeFLANN, "SHOT1344");
    defineKdtreeKdTreeFLANN<pcl::SHOT352>(sub_module_KdTreeFLANN, "SHOT352");
    defineKdtreeKdTreeFLANN<pcl::ShapeContext1980>(sub_module_KdTreeFLANN, "ShapeContext1980");
    defineKdtreeKdTreeFLANN<pcl::UniqueShapeContext1960>(sub_module_KdTreeFLANN, "UniqueShapeContext1960");
    defineKdtreeKdTreeFLANN<pcl::VFHSignature308>(sub_module_KdTreeFLANN, "VFHSignature308");
}