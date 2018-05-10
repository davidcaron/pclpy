
#include <pcl/kdtree/kdtree.h>



template <typename PointT>
void defineKdtreeKdTree(py::module &m, std::string const & suffix) {
    using Class = pcl::KdTree<PointT>;
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
    cls.def("nearestKSearch", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "p_q"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<const PointCloud &, int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "cloud"_a, "index"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<int, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "index"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("radiusSearch", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "p_q"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<const PointCloud &, int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "cloud"_a, "index"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<int, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "index"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a, "indices"_a=pcl::IndicesConstPtr());
    cls.def("setPointRepresentation", &Class::setPointRepresentation, "point_representation"_a);
    cls.def("setEpsilon", &Class::setEpsilon, "eps"_a);
    cls.def("setMinPts", &Class::setMinPts, "min_pts"_a);
    cls.def("getIndices", &Class::getIndices);
    cls.def("getInputCloud", &Class::getInputCloud);
    cls.def("getPointRepresentation", &Class::getPointRepresentation);
    cls.def("getEpsilon", &Class::getEpsilon);
    cls.def("getMinPts", &Class::getMinPts);
    cls.def("nearestKSearchT", py::overload_cast<const pcl::PointXYZ &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearchT<pcl::PointXYZ>, py::const_), "point"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("radiusSearchT", py::overload_cast<const pcl::PointXYZ &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearchT<pcl::PointXYZ>, py::const_), "point"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
        
}

void defineKdtreeKdtreeFunctions(py::module &m) {
}

void defineKdtreeKdtreeClasses(py::module &sub_module) {
    py::module sub_module_KdTree = sub_module.def_submodule("KdTree", "Submodule KdTree");
    defineKdtreeKdTree<pcl::Axis>(sub_module_KdTree, "Axis");
    defineKdtreeKdTree<pcl::BRISKSignature512>(sub_module_KdTree, "BRISKSignature512");
    defineKdtreeKdTree<pcl::Boundary>(sub_module_KdTree, "Boundary");
    defineKdtreeKdTree<pcl::CPPFSignature>(sub_module_KdTree, "CPPFSignature");
    defineKdtreeKdTree<pcl::ESFSignature640>(sub_module_KdTree, "ESFSignature640");
    defineKdtreeKdTree<pcl::FPFHSignature33>(sub_module_KdTree, "FPFHSignature33");
    defineKdtreeKdTree<pcl::GRSDSignature21>(sub_module_KdTree, "GRSDSignature21");
    defineKdtreeKdTree<pcl::IntensityGradient>(sub_module_KdTree, "IntensityGradient");
    defineKdtreeKdTree<pcl::InterestPoint>(sub_module_KdTree, "InterestPoint");
    defineKdtreeKdTree<pcl::Label>(sub_module_KdTree, "Label");
    defineKdtreeKdTree<pcl::MomentInvariants>(sub_module_KdTree, "MomentInvariants");
    defineKdtreeKdTree<pcl::Narf36>(sub_module_KdTree, "Narf36");
    defineKdtreeKdTree<pcl::Normal>(sub_module_KdTree, "Normal");
    defineKdtreeKdTree<pcl::NormalBasedSignature12>(sub_module_KdTree, "NormalBasedSignature12");
    defineKdtreeKdTree<pcl::PFHRGBSignature250>(sub_module_KdTree, "PFHRGBSignature250");
    defineKdtreeKdTree<pcl::PFHSignature125>(sub_module_KdTree, "PFHSignature125");
    defineKdtreeKdTree<pcl::PPFRGBSignature>(sub_module_KdTree, "PPFRGBSignature");
    defineKdtreeKdTree<pcl::PPFSignature>(sub_module_KdTree, "PPFSignature");
    defineKdtreeKdTree<pcl::PointDEM>(sub_module_KdTree, "PointDEM");
    defineKdtreeKdTree<pcl::PointNormal>(sub_module_KdTree, "PointNormal");
    defineKdtreeKdTree<pcl::PointSurfel>(sub_module_KdTree, "PointSurfel");
    defineKdtreeKdTree<pcl::PointUV>(sub_module_KdTree, "PointUV");
    defineKdtreeKdTree<pcl::PointWithRange>(sub_module_KdTree, "PointWithRange");
    defineKdtreeKdTree<pcl::PointWithScale>(sub_module_KdTree, "PointWithScale");
    defineKdtreeKdTree<pcl::PointWithViewpoint>(sub_module_KdTree, "PointWithViewpoint");
    defineKdtreeKdTree<pcl::PointXY>(sub_module_KdTree, "PointXY");
    defineKdtreeKdTree<pcl::PointXYZ>(sub_module_KdTree, "PointXYZ");
    defineKdtreeKdTree<pcl::PointXYZHSV>(sub_module_KdTree, "PointXYZHSV");
    defineKdtreeKdTree<pcl::PointXYZI>(sub_module_KdTree, "PointXYZI");
    defineKdtreeKdTree<pcl::PointXYZINormal>(sub_module_KdTree, "PointXYZINormal");
    defineKdtreeKdTree<pcl::PointXYZL>(sub_module_KdTree, "PointXYZL");
    defineKdtreeKdTree<pcl::PointXYZLNormal>(sub_module_KdTree, "PointXYZLNormal");
    defineKdtreeKdTree<pcl::PointXYZRGB>(sub_module_KdTree, "PointXYZRGB");
    defineKdtreeKdTree<pcl::PointXYZRGBA>(sub_module_KdTree, "PointXYZRGBA");
    defineKdtreeKdTree<pcl::PointXYZRGBL>(sub_module_KdTree, "PointXYZRGBL");
    defineKdtreeKdTree<pcl::PointXYZRGBNormal>(sub_module_KdTree, "PointXYZRGBNormal");
    defineKdtreeKdTree<pcl::PrincipalCurvatures>(sub_module_KdTree, "PrincipalCurvatures");
    defineKdtreeKdTree<pcl::PrincipalRadiiRSD>(sub_module_KdTree, "PrincipalRadiiRSD");
    defineKdtreeKdTree<pcl::ReferenceFrame>(sub_module_KdTree, "ReferenceFrame");
    defineKdtreeKdTree<pcl::SHOT1344>(sub_module_KdTree, "SHOT1344");
    defineKdtreeKdTree<pcl::SHOT352>(sub_module_KdTree, "SHOT352");
    defineKdtreeKdTree<pcl::ShapeContext1980>(sub_module_KdTree, "ShapeContext1980");
    defineKdtreeKdTree<pcl::UniqueShapeContext1960>(sub_module_KdTree, "UniqueShapeContext1960");
    defineKdtreeKdTree<pcl::VFHSignature308>(sub_module_KdTree, "VFHSignature308");
}