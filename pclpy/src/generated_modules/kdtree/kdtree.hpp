
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/kdtree/kdtree.h>



template <typename PointT>
void defineKdtreeKdTree(py::module &m, std::string const & suffix) {
    using Class = KdTree<PointT>;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointRepresentation = Class::PointRepresentation;
    using PointRepresentationConstPtr = Class::PointRepresentationConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("input_cloud", &Class::getInputCloud, &Class::setInputCloud);
    cls.def_property("point_representation", &Class::getPointRepresentation, &Class::setPointRepresentation);
    cls.def_property("epsilon", &Class::getEpsilon, &Class::setEpsilon);
    cls.def_property("min_pts", &Class::getMinPts, &Class::setMinPts);
    cls.def("nearest_k_search", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("nearest_k_search", py::overload_cast<const PointCloud &, int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("nearest_k_search", py::overload_cast<int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointCloud &, int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("radius_search", py::overload_cast<int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
        
}

void defineKdtreeKdtreeClasses(py::module &sub_module) {
    py::module sub_module_KdTree = sub_module.def_submodule("KdTree", "Submodule KdTree");
    defineKdtreeKdTree<Axis>(sub_module_KdTree, "Axis");
    defineKdtreeKdTree<BRISKSignature512>(sub_module_KdTree, "BRISKSignature512");
    defineKdtreeKdTree<Boundary>(sub_module_KdTree, "Boundary");
    defineKdtreeKdTree<CPPFSignature>(sub_module_KdTree, "CPPFSignature");
    defineKdtreeKdTree<ESFSignature640>(sub_module_KdTree, "ESFSignature640");
    defineKdtreeKdTree<FPFHSignature33>(sub_module_KdTree, "FPFHSignature33");
    defineKdtreeKdTree<GRSDSignature21>(sub_module_KdTree, "GRSDSignature21");
    defineKdtreeKdTree<IntensityGradient>(sub_module_KdTree, "IntensityGradient");
    defineKdtreeKdTree<InterestPoint>(sub_module_KdTree, "InterestPoint");
    defineKdtreeKdTree<Label>(sub_module_KdTree, "Label");
    defineKdtreeKdTree<MomentInvariants>(sub_module_KdTree, "MomentInvariants");
    defineKdtreeKdTree<Narf36>(sub_module_KdTree, "Narf36");
    defineKdtreeKdTree<Normal>(sub_module_KdTree, "Normal");
    defineKdtreeKdTree<NormalBasedSignature12>(sub_module_KdTree, "NormalBasedSignature12");
    defineKdtreeKdTree<PFHRGBSignature250>(sub_module_KdTree, "PFHRGBSignature250");
    defineKdtreeKdTree<PFHSignature125>(sub_module_KdTree, "PFHSignature125");
    defineKdtreeKdTree<PPFRGBSignature>(sub_module_KdTree, "PPFRGBSignature");
    defineKdtreeKdTree<PPFSignature>(sub_module_KdTree, "PPFSignature");
    defineKdtreeKdTree<PointDEM>(sub_module_KdTree, "PointDEM");
    defineKdtreeKdTree<PointNormal>(sub_module_KdTree, "PointNormal");
    defineKdtreeKdTree<PointSurfel>(sub_module_KdTree, "PointSurfel");
    defineKdtreeKdTree<PointUV>(sub_module_KdTree, "PointUV");
    defineKdtreeKdTree<PointWithRange>(sub_module_KdTree, "PointWithRange");
    defineKdtreeKdTree<PointWithScale>(sub_module_KdTree, "PointWithScale");
    defineKdtreeKdTree<PointWithViewpoint>(sub_module_KdTree, "PointWithViewpoint");
    defineKdtreeKdTree<PointXY>(sub_module_KdTree, "PointXY");
    defineKdtreeKdTree<PointXYZ>(sub_module_KdTree, "PointXYZ");
    defineKdtreeKdTree<PointXYZHSV>(sub_module_KdTree, "PointXYZHSV");
    defineKdtreeKdTree<PointXYZI>(sub_module_KdTree, "PointXYZI");
    defineKdtreeKdTree<PointXYZINormal>(sub_module_KdTree, "PointXYZINormal");
    defineKdtreeKdTree<PointXYZL>(sub_module_KdTree, "PointXYZL");
    defineKdtreeKdTree<PointXYZLNormal>(sub_module_KdTree, "PointXYZLNormal");
    defineKdtreeKdTree<PointXYZRGB>(sub_module_KdTree, "PointXYZRGB");
    defineKdtreeKdTree<PointXYZRGBA>(sub_module_KdTree, "PointXYZRGBA");
    defineKdtreeKdTree<PointXYZRGBL>(sub_module_KdTree, "PointXYZRGBL");
    defineKdtreeKdTree<PointXYZRGBNormal>(sub_module_KdTree, "PointXYZRGBNormal");
    defineKdtreeKdTree<PrincipalCurvatures>(sub_module_KdTree, "PrincipalCurvatures");
    defineKdtreeKdTree<PrincipalRadiiRSD>(sub_module_KdTree, "PrincipalRadiiRSD");
    defineKdtreeKdTree<ReferenceFrame>(sub_module_KdTree, "ReferenceFrame");
    defineKdtreeKdTree<SHOT1344>(sub_module_KdTree, "SHOT1344");
    defineKdtreeKdTree<SHOT352>(sub_module_KdTree, "SHOT352");
    defineKdtreeKdTree<ShapeContext1980>(sub_module_KdTree, "ShapeContext1980");
    defineKdtreeKdTree<UniqueShapeContext1960>(sub_module_KdTree, "UniqueShapeContext1960");
    defineKdtreeKdTree<VFHSignature308>(sub_module_KdTree, "VFHSignature308");
}