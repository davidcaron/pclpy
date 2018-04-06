
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/kdtree/kdtree_flann.h>



template <typename PointT, typename Dist = ::flann::L2_Simple<float> >
void defineKdtreeKdTreeFLANN(py::module &m, std::string const & suffix) {
    using Class = KdTreeFLANN<PointT, Dist>;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using FLANNIndex = Class::FLANNIndex;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, KdTree<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "sorted"_a=true);
    cls.def("set_epsilon", &Class::setEpsilon);
    cls.def("set_sorted_results", &Class::setSortedResults);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("nearest_k_search", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    // Operators not implemented (operator=);
    cls.def("make_shared", &Class::makeShared);
        
}

void defineKdtreeKdtreeFlannClasses(py::module &sub_module) {
    py::module sub_module_KdTreeFLANN = sub_module.def_submodule("KdTreeFLANN", "Submodule KdTreeFLANN");
    defineKdtreeKdTreeFLANN<Axis>(sub_module_KdTreeFLANN, "Axis");
    defineKdtreeKdTreeFLANN<BRISKSignature512>(sub_module_KdTreeFLANN, "BRISKSignature512");
    defineKdtreeKdTreeFLANN<Boundary>(sub_module_KdTreeFLANN, "Boundary");
    defineKdtreeKdTreeFLANN<CPPFSignature>(sub_module_KdTreeFLANN, "CPPFSignature");
    defineKdtreeKdTreeFLANN<ESFSignature640>(sub_module_KdTreeFLANN, "ESFSignature640");
    defineKdtreeKdTreeFLANN<FPFHSignature33>(sub_module_KdTreeFLANN, "FPFHSignature33");
    defineKdtreeKdTreeFLANN<GRSDSignature21>(sub_module_KdTreeFLANN, "GRSDSignature21");
    defineKdtreeKdTreeFLANN<IntensityGradient>(sub_module_KdTreeFLANN, "IntensityGradient");
    defineKdtreeKdTreeFLANN<InterestPoint>(sub_module_KdTreeFLANN, "InterestPoint");
    defineKdtreeKdTreeFLANN<Label>(sub_module_KdTreeFLANN, "Label");
    defineKdtreeKdTreeFLANN<MomentInvariants>(sub_module_KdTreeFLANN, "MomentInvariants");
    defineKdtreeKdTreeFLANN<Narf36>(sub_module_KdTreeFLANN, "Narf36");
    defineKdtreeKdTreeFLANN<Normal>(sub_module_KdTreeFLANN, "Normal");
    defineKdtreeKdTreeFLANN<NormalBasedSignature12>(sub_module_KdTreeFLANN, "NormalBasedSignature12");
    defineKdtreeKdTreeFLANN<PFHRGBSignature250>(sub_module_KdTreeFLANN, "PFHRGBSignature250");
    defineKdtreeKdTreeFLANN<PFHSignature125>(sub_module_KdTreeFLANN, "PFHSignature125");
    defineKdtreeKdTreeFLANN<PPFRGBSignature>(sub_module_KdTreeFLANN, "PPFRGBSignature");
    defineKdtreeKdTreeFLANN<PPFSignature>(sub_module_KdTreeFLANN, "PPFSignature");
    defineKdtreeKdTreeFLANN<PointDEM>(sub_module_KdTreeFLANN, "PointDEM");
    defineKdtreeKdTreeFLANN<PointNormal>(sub_module_KdTreeFLANN, "PointNormal");
    defineKdtreeKdTreeFLANN<PointSurfel>(sub_module_KdTreeFLANN, "PointSurfel");
    defineKdtreeKdTreeFLANN<PointUV>(sub_module_KdTreeFLANN, "PointUV");
    defineKdtreeKdTreeFLANN<PointWithRange>(sub_module_KdTreeFLANN, "PointWithRange");
    defineKdtreeKdTreeFLANN<PointWithScale>(sub_module_KdTreeFLANN, "PointWithScale");
    defineKdtreeKdTreeFLANN<PointWithViewpoint>(sub_module_KdTreeFLANN, "PointWithViewpoint");
    defineKdtreeKdTreeFLANN<PointXY>(sub_module_KdTreeFLANN, "PointXY");
    defineKdtreeKdTreeFLANN<PointXYZ>(sub_module_KdTreeFLANN, "PointXYZ");
    defineKdtreeKdTreeFLANN<PointXYZHSV>(sub_module_KdTreeFLANN, "PointXYZHSV");
    defineKdtreeKdTreeFLANN<PointXYZI>(sub_module_KdTreeFLANN, "PointXYZI");
    defineKdtreeKdTreeFLANN<PointXYZINormal>(sub_module_KdTreeFLANN, "PointXYZINormal");
    defineKdtreeKdTreeFLANN<PointXYZL>(sub_module_KdTreeFLANN, "PointXYZL");
    defineKdtreeKdTreeFLANN<PointXYZLNormal>(sub_module_KdTreeFLANN, "PointXYZLNormal");
    defineKdtreeKdTreeFLANN<PointXYZRGB>(sub_module_KdTreeFLANN, "PointXYZRGB");
    defineKdtreeKdTreeFLANN<PointXYZRGBA>(sub_module_KdTreeFLANN, "PointXYZRGBA");
    defineKdtreeKdTreeFLANN<PointXYZRGBL>(sub_module_KdTreeFLANN, "PointXYZRGBL");
    defineKdtreeKdTreeFLANN<PointXYZRGBNormal>(sub_module_KdTreeFLANN, "PointXYZRGBNormal");
    defineKdtreeKdTreeFLANN<PrincipalCurvatures>(sub_module_KdTreeFLANN, "PrincipalCurvatures");
    defineKdtreeKdTreeFLANN<PrincipalRadiiRSD>(sub_module_KdTreeFLANN, "PrincipalRadiiRSD");
    defineKdtreeKdTreeFLANN<ReferenceFrame>(sub_module_KdTreeFLANN, "ReferenceFrame");
    defineKdtreeKdTreeFLANN<SHOT1344>(sub_module_KdTreeFLANN, "SHOT1344");
    defineKdtreeKdTreeFLANN<SHOT352>(sub_module_KdTreeFLANN, "SHOT352");
    defineKdtreeKdTreeFLANN<ShapeContext1980>(sub_module_KdTreeFLANN, "ShapeContext1980");
    defineKdtreeKdTreeFLANN<UniqueShapeContext1960>(sub_module_KdTreeFLANN, "UniqueShapeContext1960");
    defineKdtreeKdTreeFLANN<VFHSignature308>(sub_module_KdTreeFLANN, "VFHSignature308");
}