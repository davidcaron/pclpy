
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/search/kdtree.h>

using namespace pcl::search;


template<typename PointT, class Tree = pcl::KdTreeFLANN<PointT> >
void defineSearchKdTree(py::module &m, std::string const & suffix) {
    using Class = search::KdTree<PointT, Tree>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using KdTreePtr = Class::KdTreePtr;
    using KdTreeConstPtr = Class::KdTreeConstPtr;
    using PointRepresentationConstPtr = Class::PointRepresentationConstPtr;
    py::class_<Class, Search<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "sorted"_a=true);
    cls.def_property("point_representation", &Class::getPointRepresentation, &Class::setPointRepresentation);
    cls.def("set_sorted_results", &Class::setSortedResults);
    cls.def_property("epsilon", &Class::getEpsilon, &Class::setEpsilon);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("nearest_k_search", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
        
}

void defineSearchKdtreeClasses(py::module &sub_module) {
    py::module sub_module_KdTree = sub_module.def_submodule("KdTree", "Submodule KdTree");
    defineSearchKdTree<Axis>(sub_module_KdTree, "Axis");
    defineSearchKdTree<BRISKSignature512>(sub_module_KdTree, "BRISKSignature512");
    defineSearchKdTree<Boundary>(sub_module_KdTree, "Boundary");
    defineSearchKdTree<CPPFSignature>(sub_module_KdTree, "CPPFSignature");
    defineSearchKdTree<ESFSignature640>(sub_module_KdTree, "ESFSignature640");
    defineSearchKdTree<FPFHSignature33>(sub_module_KdTree, "FPFHSignature33");
    defineSearchKdTree<GRSDSignature21>(sub_module_KdTree, "GRSDSignature21");
    defineSearchKdTree<IntensityGradient>(sub_module_KdTree, "IntensityGradient");
    defineSearchKdTree<InterestPoint>(sub_module_KdTree, "InterestPoint");
    defineSearchKdTree<Label>(sub_module_KdTree, "Label");
    defineSearchKdTree<MomentInvariants>(sub_module_KdTree, "MomentInvariants");
    defineSearchKdTree<Narf36>(sub_module_KdTree, "Narf36");
    defineSearchKdTree<Normal>(sub_module_KdTree, "Normal");
    defineSearchKdTree<NormalBasedSignature12>(sub_module_KdTree, "NormalBasedSignature12");
    defineSearchKdTree<PFHRGBSignature250>(sub_module_KdTree, "PFHRGBSignature250");
    defineSearchKdTree<PFHSignature125>(sub_module_KdTree, "PFHSignature125");
    defineSearchKdTree<PPFRGBSignature>(sub_module_KdTree, "PPFRGBSignature");
    defineSearchKdTree<PPFSignature>(sub_module_KdTree, "PPFSignature");
    defineSearchKdTree<PointDEM>(sub_module_KdTree, "PointDEM");
    defineSearchKdTree<PointNormal>(sub_module_KdTree, "PointNormal");
    defineSearchKdTree<PointSurfel>(sub_module_KdTree, "PointSurfel");
    defineSearchKdTree<PointUV>(sub_module_KdTree, "PointUV");
    defineSearchKdTree<PointWithRange>(sub_module_KdTree, "PointWithRange");
    defineSearchKdTree<PointWithScale>(sub_module_KdTree, "PointWithScale");
    defineSearchKdTree<PointWithViewpoint>(sub_module_KdTree, "PointWithViewpoint");
    defineSearchKdTree<PointXY>(sub_module_KdTree, "PointXY");
    defineSearchKdTree<PointXYZ>(sub_module_KdTree, "PointXYZ");
    defineSearchKdTree<PointXYZHSV>(sub_module_KdTree, "PointXYZHSV");
    defineSearchKdTree<PointXYZI>(sub_module_KdTree, "PointXYZI");
    defineSearchKdTree<PointXYZINormal>(sub_module_KdTree, "PointXYZINormal");
    defineSearchKdTree<PointXYZL>(sub_module_KdTree, "PointXYZL");
    defineSearchKdTree<PointXYZLNormal>(sub_module_KdTree, "PointXYZLNormal");
    defineSearchKdTree<PointXYZRGB>(sub_module_KdTree, "PointXYZRGB");
    defineSearchKdTree<PointXYZRGBA>(sub_module_KdTree, "PointXYZRGBA");
    defineSearchKdTree<PointXYZRGBL>(sub_module_KdTree, "PointXYZRGBL");
    defineSearchKdTree<PointXYZRGBNormal>(sub_module_KdTree, "PointXYZRGBNormal");
    defineSearchKdTree<PrincipalCurvatures>(sub_module_KdTree, "PrincipalCurvatures");
    defineSearchKdTree<PrincipalRadiiRSD>(sub_module_KdTree, "PrincipalRadiiRSD");
    defineSearchKdTree<ReferenceFrame>(sub_module_KdTree, "ReferenceFrame");
    defineSearchKdTree<SHOT1344>(sub_module_KdTree, "SHOT1344");
    defineSearchKdTree<SHOT352>(sub_module_KdTree, "SHOT352");
    defineSearchKdTree<ShapeContext1980>(sub_module_KdTree, "ShapeContext1980");
    defineSearchKdTree<UniqueShapeContext1960>(sub_module_KdTree, "UniqueShapeContext1960");
    defineSearchKdTree<VFHSignature308>(sub_module_KdTree, "VFHSignature308");
}