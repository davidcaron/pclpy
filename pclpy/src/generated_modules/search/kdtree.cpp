
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/search/kdtree.h>

using namespace pcl::search;


template<typename PointT, class Tree = pcl::KdTreeFLANN<PointT> >
void defineSearchKdTree(py::module &m, std::string const & suffix) {
    using Class = pcl::search::KdTree<PointT, Tree>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using KdTreePtr = Class::KdTreePtr;
    using KdTreeConstPtr = Class::KdTreeConstPtr;
    using PointRepresentationConstPtr = Class::PointRepresentationConstPtr;
    py::class_<Class, pcl::search::Search<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "sorted"_a=true);
    cls.def("nearestKSearch", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "point"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("radiusSearch", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "point"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("setPointRepresentation", &Class::setPointRepresentation, "point_representation"_a);
    cls.def("setSortedResults", &Class::setSortedResults, "sorted_results"_a);
    cls.def("setEpsilon", &Class::setEpsilon, "eps"_a);
    cls.def("setInputCloud", py::overload_cast<const PointCloudConstPtr &, const IndicesConstPtr &> (&Class::setInputCloud), "cloud"_a, "indices"_a=pcl::IndicesConstPtr());
    cls.def("getPointRepresentation", &Class::getPointRepresentation);
    cls.def("getEpsilon", &Class::getEpsilon);
        
}

void defineSearchKdtreeFunctions(py::module &m) {
}

void defineSearchKdtreeClasses(py::module &sub_module) {
    py::module sub_module_KdTree = sub_module.def_submodule("KdTree", "Submodule KdTree");
    defineSearchKdTree<pcl::Axis>(sub_module_KdTree, "Axis");
    defineSearchKdTree<pcl::BRISKSignature512>(sub_module_KdTree, "BRISKSignature512");
    defineSearchKdTree<pcl::Boundary>(sub_module_KdTree, "Boundary");
    defineSearchKdTree<pcl::CPPFSignature>(sub_module_KdTree, "CPPFSignature");
    defineSearchKdTree<pcl::ESFSignature640>(sub_module_KdTree, "ESFSignature640");
    defineSearchKdTree<pcl::FPFHSignature33>(sub_module_KdTree, "FPFHSignature33");
    defineSearchKdTree<pcl::GRSDSignature21>(sub_module_KdTree, "GRSDSignature21");
    defineSearchKdTree<pcl::IntensityGradient>(sub_module_KdTree, "IntensityGradient");
    defineSearchKdTree<pcl::InterestPoint>(sub_module_KdTree, "InterestPoint");
    defineSearchKdTree<pcl::Label>(sub_module_KdTree, "Label");
    defineSearchKdTree<pcl::MomentInvariants>(sub_module_KdTree, "MomentInvariants");
    defineSearchKdTree<pcl::Narf36>(sub_module_KdTree, "Narf36");
    defineSearchKdTree<pcl::Normal>(sub_module_KdTree, "Normal");
    defineSearchKdTree<pcl::NormalBasedSignature12>(sub_module_KdTree, "NormalBasedSignature12");
    defineSearchKdTree<pcl::PFHRGBSignature250>(sub_module_KdTree, "PFHRGBSignature250");
    defineSearchKdTree<pcl::PFHSignature125>(sub_module_KdTree, "PFHSignature125");
    defineSearchKdTree<pcl::PPFRGBSignature>(sub_module_KdTree, "PPFRGBSignature");
    defineSearchKdTree<pcl::PPFSignature>(sub_module_KdTree, "PPFSignature");
    defineSearchKdTree<pcl::PointDEM>(sub_module_KdTree, "PointDEM");
    defineSearchKdTree<pcl::PointNormal>(sub_module_KdTree, "PointNormal");
    defineSearchKdTree<pcl::PointSurfel>(sub_module_KdTree, "PointSurfel");
    defineSearchKdTree<pcl::PointUV>(sub_module_KdTree, "PointUV");
    defineSearchKdTree<pcl::PointWithRange>(sub_module_KdTree, "PointWithRange");
    defineSearchKdTree<pcl::PointWithScale>(sub_module_KdTree, "PointWithScale");
    defineSearchKdTree<pcl::PointWithViewpoint>(sub_module_KdTree, "PointWithViewpoint");
    defineSearchKdTree<pcl::PointXY>(sub_module_KdTree, "PointXY");
    defineSearchKdTree<pcl::PointXYZ>(sub_module_KdTree, "PointXYZ");
    defineSearchKdTree<pcl::PointXYZHSV>(sub_module_KdTree, "PointXYZHSV");
    defineSearchKdTree<pcl::PointXYZI>(sub_module_KdTree, "PointXYZI");
    defineSearchKdTree<pcl::PointXYZINormal>(sub_module_KdTree, "PointXYZINormal");
    defineSearchKdTree<pcl::PointXYZL>(sub_module_KdTree, "PointXYZL");
    defineSearchKdTree<pcl::PointXYZLNormal>(sub_module_KdTree, "PointXYZLNormal");
    defineSearchKdTree<pcl::PointXYZRGB>(sub_module_KdTree, "PointXYZRGB");
    defineSearchKdTree<pcl::PointXYZRGBA>(sub_module_KdTree, "PointXYZRGBA");
    defineSearchKdTree<pcl::PointXYZRGBL>(sub_module_KdTree, "PointXYZRGBL");
    defineSearchKdTree<pcl::PointXYZRGBNormal>(sub_module_KdTree, "PointXYZRGBNormal");
    defineSearchKdTree<pcl::PrincipalCurvatures>(sub_module_KdTree, "PrincipalCurvatures");
    defineSearchKdTree<pcl::PrincipalRadiiRSD>(sub_module_KdTree, "PrincipalRadiiRSD");
    defineSearchKdTree<pcl::ReferenceFrame>(sub_module_KdTree, "ReferenceFrame");
    defineSearchKdTree<pcl::SHOT1344>(sub_module_KdTree, "SHOT1344");
    defineSearchKdTree<pcl::SHOT352>(sub_module_KdTree, "SHOT352");
    defineSearchKdTree<pcl::ShapeContext1980>(sub_module_KdTree, "ShapeContext1980");
    defineSearchKdTree<pcl::UniqueShapeContext1960>(sub_module_KdTree, "UniqueShapeContext1960");
    defineSearchKdTree<pcl::VFHSignature308>(sub_module_KdTree, "VFHSignature308");
}