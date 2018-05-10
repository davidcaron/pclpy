
#include <pcl/features/feature.h>



template <typename PointInT, typename PointRFT>
void defineFeaturesFeatureWithLocalReferenceFrames(py::module &m, std::string const & suffix) {
    using Class = pcl::FeatureWithLocalReferenceFrames<PointInT, PointRFT>;
    using PointCloudLRF = Class::PointCloudLRF;
    using PointCloudLRFPtr = Class::PointCloudLRFPtr;
    using PointCloudLRFConstPtr = Class::PointCloudLRFConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setInputReferenceFrames", &Class::setInputReferenceFrames, "frames"_a);
    cls.def("getInputReferenceFrames", &Class::getInputReferenceFrames);
        
}

template <typename PointInT, typename PointOutT>
void defineFeaturesFeature(py::module &m, std::string const & suffix) {
    using Class = pcl::Feature<PointInT, PointOutT>;
    using BaseClass = Class::BaseClass;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("compute", &Class::compute, "output"_a);
    cls.def("setSearchSurface", &Class::setSearchSurface, "cloud"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("setKSearch", &Class::setKSearch, "k"_a);
    cls.def("setRadiusSearch", &Class::setRadiusSearch, "radius"_a);
    cls.def("getSearchSurface", &Class::getSearchSurface);
    cls.def("getSearchMethod", &Class::getSearchMethod);
    cls.def("getSearchParameter", &Class::getSearchParameter);
    cls.def("getKSearch", &Class::getKSearch);
    cls.def("getRadiusSearch", &Class::getRadiusSearch);
        
}

template <typename PointInT, typename PointLT, typename PointOutT>
void defineFeaturesFeatureFromLabels(py::module &m, std::string const & suffix) {
    using Class = pcl::FeatureFromLabels<PointInT, PointLT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("setInputLabels", &Class::setInputLabels, "labels"_a);
    cls.def("getInputLabels", &Class::getInputLabels);
        
}

template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesFeatureFromNormals(py::module &m, std::string const & suffix) {
    using Class = pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("setInputNormals", &Class::setInputNormals, "normals"_a);
    cls.def("getInputNormals", &Class::getInputNormals);
        
}

void defineFeaturesFeatureFunctions1(py::module &m) {
    m.def("solvePlaneParameters", py::overload_cast<const Eigen::Matrix3f &, const Eigen::Vector4f &, Eigen::Vector4f &, float &> (&pcl::solvePlaneParameters), "covariance_matrix"_a, "point"_a, "plane_parameters"_a, "curvature"_a);
    m.def("solvePlaneParameters", py::overload_cast<const Eigen::Matrix3f &, float &, float &, float &, float &> (&pcl::solvePlaneParameters), "covariance_matrix"_a, "nx"_a, "ny"_a, "nz"_a, "curvature"_a);
}

void defineFeaturesFeatureFunctions(py::module &m) {
    defineFeaturesFeatureFunctions1(m);
}

void defineFeaturesFeatureClasses(py::module &sub_module) {
    py::module sub_module_FeatureWithLocalReferenceFrames = sub_module.def_submodule("FeatureWithLocalReferenceFrames", "Submodule FeatureWithLocalReferenceFrames");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZ, pcl::Normal>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZ_Normal");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZ, pcl::ReferenceFrame>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZ_ReferenceFrame");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZ, pcl::UniqueShapeContext1960>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZ_UniqueShapeContext1960");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZI, pcl::Normal>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZI_Normal");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZI, pcl::ReferenceFrame>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZI_ReferenceFrame");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZI, pcl::UniqueShapeContext1960>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZI_UniqueShapeContext1960");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZRGB, pcl::Normal>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZRGB_Normal");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZRGB, pcl::ReferenceFrame>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZRGB_ReferenceFrame");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZRGBA, pcl::Normal>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZRGBA_Normal");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZRGBA, pcl::ReferenceFrame>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZRGBA_ReferenceFrame");
    defineFeaturesFeatureWithLocalReferenceFrames<pcl::PointXYZRGBA, pcl::UniqueShapeContext1960>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZRGBA_UniqueShapeContext1960");
    py::module sub_module_Feature = sub_module.def_submodule("Feature", "Submodule Feature");
    defineFeaturesFeature<pcl::PointNormal, pcl::Boundary>(sub_module_Feature, "PointNormal_Boundary");
    defineFeaturesFeature<pcl::PointNormal, pcl::ESFSignature640>(sub_module_Feature, "PointNormal_ESFSignature640");
    defineFeaturesFeature<pcl::PointNormal, pcl::FPFHSignature33>(sub_module_Feature, "PointNormal_FPFHSignature33");
    defineFeaturesFeature<pcl::PointNormal, pcl::Histogram<135>>(sub_module_Feature, "PointNormal_Histogram<135>");
    defineFeaturesFeature<pcl::PointNormal, pcl::Histogram<153>>(sub_module_Feature, "PointNormal_Histogram<153>");
    defineFeaturesFeature<pcl::PointNormal, pcl::Normal>(sub_module_Feature, "PointNormal_Normal");
    defineFeaturesFeature<pcl::PointNormal, pcl::PPFSignature>(sub_module_Feature, "PointNormal_PPFSignature");
    defineFeaturesFeature<pcl::PointNormal, pcl::PointNormal>(sub_module_Feature, "PointNormal_PointNormal");
    defineFeaturesFeature<pcl::PointNormal, pcl::PointXYZRGBNormal>(sub_module_Feature, "PointNormal_PointXYZRGBNormal");
    defineFeaturesFeature<pcl::PointNormal, pcl::VFHSignature308>(sub_module_Feature, "PointNormal_VFHSignature308");
    defineFeaturesFeature<pcl::PointSurfel, pcl::Normal>(sub_module_Feature, "PointSurfel_Normal");
    defineFeaturesFeature<pcl::PointSurfel, pcl::PointNormal>(sub_module_Feature, "PointSurfel_PointNormal");
    defineFeaturesFeature<pcl::PointSurfel, pcl::PointXYZRGBNormal>(sub_module_Feature, "PointSurfel_PointXYZRGBNormal");
    defineFeaturesFeature<pcl::PointXYZ, pcl::Boundary>(sub_module_Feature, "PointXYZ_Boundary");
    defineFeaturesFeature<pcl::PointXYZ, pcl::ESFSignature640>(sub_module_Feature, "PointXYZ_ESFSignature640");
    defineFeaturesFeature<pcl::PointXYZ, pcl::FPFHSignature33>(sub_module_Feature, "PointXYZ_FPFHSignature33");
    defineFeaturesFeature<pcl::PointXYZ, pcl::GFPFHSignature16>(sub_module_Feature, "PointXYZ_GFPFHSignature16");
    defineFeaturesFeature<pcl::PointXYZ, pcl::GRSDSignature21>(sub_module_Feature, "PointXYZ_GRSDSignature21");
    defineFeaturesFeature<pcl::PointXYZ, pcl::Histogram<135>>(sub_module_Feature, "PointXYZ_Histogram<135>");
    defineFeaturesFeature<pcl::PointXYZ, pcl::Histogram<153>>(sub_module_Feature, "PointXYZ_Histogram<153>");
    defineFeaturesFeature<pcl::PointXYZ, pcl::Histogram<90>>(sub_module_Feature, "PointXYZ_Histogram<90>");
    defineFeaturesFeature<pcl::PointXYZ, pcl::MomentInvariants>(sub_module_Feature, "PointXYZ_MomentInvariants");
    defineFeaturesFeature<pcl::PointXYZ, pcl::Normal>(sub_module_Feature, "PointXYZ_Normal");
    defineFeaturesFeature<pcl::PointXYZ, pcl::NormalBasedSignature12>(sub_module_Feature, "PointXYZ_NormalBasedSignature12");
    defineFeaturesFeature<pcl::PointXYZ, pcl::PFHSignature125>(sub_module_Feature, "PointXYZ_PFHSignature125");
    defineFeaturesFeature<pcl::PointXYZ, pcl::PPFSignature>(sub_module_Feature, "PointXYZ_PPFSignature");
    defineFeaturesFeature<pcl::PointXYZ, pcl::PointNormal>(sub_module_Feature, "PointXYZ_PointNormal");
    defineFeaturesFeature<pcl::PointXYZ, pcl::PointXYZL>(sub_module_Feature, "PointXYZ_PointXYZL");
    defineFeaturesFeature<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_Feature, "PointXYZ_PointXYZRGBNormal");
    defineFeaturesFeature<pcl::PointXYZ, pcl::PrincipalCurvatures>(sub_module_Feature, "PointXYZ_PrincipalCurvatures");
    defineFeaturesFeature<pcl::PointXYZ, pcl::PrincipalRadiiRSD>(sub_module_Feature, "PointXYZ_PrincipalRadiiRSD");
    defineFeaturesFeature<pcl::PointXYZ, pcl::ReferenceFrame>(sub_module_Feature, "PointXYZ_ReferenceFrame");
    defineFeaturesFeature<pcl::PointXYZ, pcl::SHOT1344>(sub_module_Feature, "PointXYZ_SHOT1344");
    defineFeaturesFeature<pcl::PointXYZ, pcl::SHOT352>(sub_module_Feature, "PointXYZ_SHOT352");
    defineFeaturesFeature<pcl::PointXYZ, pcl::ShapeContext1980>(sub_module_Feature, "PointXYZ_ShapeContext1980");
    defineFeaturesFeature<pcl::PointXYZ, pcl::UniqueShapeContext1960>(sub_module_Feature, "PointXYZ_UniqueShapeContext1960");
    defineFeaturesFeature<pcl::PointXYZ, pcl::VFHSignature308>(sub_module_Feature, "PointXYZ_VFHSignature308");
    defineFeaturesFeature<pcl::PointXYZI, pcl::Boundary>(sub_module_Feature, "PointXYZI_Boundary");
    defineFeaturesFeature<pcl::PointXYZI, pcl::ESFSignature640>(sub_module_Feature, "PointXYZI_ESFSignature640");
    defineFeaturesFeature<pcl::PointXYZI, pcl::FPFHSignature33>(sub_module_Feature, "PointXYZI_FPFHSignature33");
    defineFeaturesFeature<pcl::PointXYZI, pcl::GFPFHSignature16>(sub_module_Feature, "PointXYZI_GFPFHSignature16");
    defineFeaturesFeature<pcl::PointXYZI, pcl::GRSDSignature21>(sub_module_Feature, "PointXYZI_GRSDSignature21");
    defineFeaturesFeature<pcl::PointXYZI, pcl::Histogram<135>>(sub_module_Feature, "PointXYZI_Histogram<135>");
    defineFeaturesFeature<pcl::PointXYZI, pcl::Histogram<153>>(sub_module_Feature, "PointXYZI_Histogram<153>");
    defineFeaturesFeature<pcl::PointXYZI, pcl::Histogram<20>>(sub_module_Feature, "PointXYZI_Histogram<20>");
    defineFeaturesFeature<pcl::PointXYZI, pcl::Histogram<32>>(sub_module_Feature, "PointXYZI_Histogram<32>");
    defineFeaturesFeature<pcl::PointXYZI, pcl::Histogram<90>>(sub_module_Feature, "PointXYZI_Histogram<90>");
    defineFeaturesFeature<pcl::PointXYZI, pcl::IntensityGradient>(sub_module_Feature, "PointXYZI_IntensityGradient");
    defineFeaturesFeature<pcl::PointXYZI, pcl::MomentInvariants>(sub_module_Feature, "PointXYZI_MomentInvariants");
    defineFeaturesFeature<pcl::PointXYZI, pcl::Normal>(sub_module_Feature, "PointXYZI_Normal");
    defineFeaturesFeature<pcl::PointXYZI, pcl::NormalBasedSignature12>(sub_module_Feature, "PointXYZI_NormalBasedSignature12");
    defineFeaturesFeature<pcl::PointXYZI, pcl::PFHSignature125>(sub_module_Feature, "PointXYZI_PFHSignature125");
    defineFeaturesFeature<pcl::PointXYZI, pcl::PPFSignature>(sub_module_Feature, "PointXYZI_PPFSignature");
    defineFeaturesFeature<pcl::PointXYZI, pcl::PointNormal>(sub_module_Feature, "PointXYZI_PointNormal");
    defineFeaturesFeature<pcl::PointXYZI, pcl::PointXYZL>(sub_module_Feature, "PointXYZI_PointXYZL");
    defineFeaturesFeature<pcl::PointXYZI, pcl::PointXYZRGBNormal>(sub_module_Feature, "PointXYZI_PointXYZRGBNormal");
    defineFeaturesFeature<pcl::PointXYZI, pcl::PrincipalCurvatures>(sub_module_Feature, "PointXYZI_PrincipalCurvatures");
    defineFeaturesFeature<pcl::PointXYZI, pcl::PrincipalRadiiRSD>(sub_module_Feature, "PointXYZI_PrincipalRadiiRSD");
    defineFeaturesFeature<pcl::PointXYZI, pcl::ReferenceFrame>(sub_module_Feature, "PointXYZI_ReferenceFrame");
    defineFeaturesFeature<pcl::PointXYZI, pcl::SHOT1344>(sub_module_Feature, "PointXYZI_SHOT1344");
    defineFeaturesFeature<pcl::PointXYZI, pcl::SHOT352>(sub_module_Feature, "PointXYZI_SHOT352");
    defineFeaturesFeature<pcl::PointXYZI, pcl::ShapeContext1980>(sub_module_Feature, "PointXYZI_ShapeContext1980");
    defineFeaturesFeature<pcl::PointXYZI, pcl::UniqueShapeContext1960>(sub_module_Feature, "PointXYZI_UniqueShapeContext1960");
    defineFeaturesFeature<pcl::PointXYZI, pcl::VFHSignature308>(sub_module_Feature, "PointXYZI_VFHSignature308");
    defineFeaturesFeature<pcl::PointXYZL, pcl::GFPFHSignature16>(sub_module_Feature, "PointXYZL_GFPFHSignature16");
    defineFeaturesFeature<pcl::PointXYZL, pcl::PointXYZL>(sub_module_Feature, "PointXYZL_PointXYZL");
    defineFeaturesFeature<pcl::PointXYZRGB, pcl::FPFHSignature33>(sub_module_Feature, "PointXYZRGB_FPFHSignature33");
    defineFeaturesFeature<pcl::PointXYZRGB, pcl::Normal>(sub_module_Feature, "PointXYZRGB_Normal");
    defineFeaturesFeature<pcl::PointXYZRGB, pcl::PFHRGBSignature250>(sub_module_Feature, "PointXYZRGB_PFHRGBSignature250");
    defineFeaturesFeature<pcl::PointXYZRGB, pcl::PFHSignature125>(sub_module_Feature, "PointXYZRGB_PFHSignature125");
    defineFeaturesFeature<pcl::PointXYZRGB, pcl::PointNormal>(sub_module_Feature, "PointXYZRGB_PointNormal");
    defineFeaturesFeature<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_Feature, "PointXYZRGB_PointXYZRGBNormal");
    defineFeaturesFeature<pcl::PointXYZRGB, pcl::ReferenceFrame>(sub_module_Feature, "PointXYZRGB_ReferenceFrame");
    defineFeaturesFeature<pcl::PointXYZRGB, pcl::SHOT1344>(sub_module_Feature, "PointXYZRGB_SHOT1344");
    defineFeaturesFeature<pcl::PointXYZRGB, pcl::SHOT352>(sub_module_Feature, "PointXYZRGB_SHOT352");
    defineFeaturesFeature<pcl::PointXYZRGB, pcl::VFHSignature308>(sub_module_Feature, "PointXYZRGB_VFHSignature308");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::Boundary>(sub_module_Feature, "PointXYZRGBA_Boundary");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::CPPFSignature>(sub_module_Feature, "PointXYZRGBA_CPPFSignature");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::ESFSignature640>(sub_module_Feature, "PointXYZRGBA_ESFSignature640");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::FPFHSignature33>(sub_module_Feature, "PointXYZRGBA_FPFHSignature33");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::GFPFHSignature16>(sub_module_Feature, "PointXYZRGBA_GFPFHSignature16");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::GRSDSignature21>(sub_module_Feature, "PointXYZRGBA_GRSDSignature21");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::Histogram<135>>(sub_module_Feature, "PointXYZRGBA_Histogram<135>");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::Histogram<153>>(sub_module_Feature, "PointXYZRGBA_Histogram<153>");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::Histogram<90>>(sub_module_Feature, "PointXYZRGBA_Histogram<90>");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::MomentInvariants>(sub_module_Feature, "PointXYZRGBA_MomentInvariants");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::Normal>(sub_module_Feature, "PointXYZRGBA_Normal");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::NormalBasedSignature12>(sub_module_Feature, "PointXYZRGBA_NormalBasedSignature12");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::PFHRGBSignature250>(sub_module_Feature, "PointXYZRGBA_PFHRGBSignature250");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::PFHSignature125>(sub_module_Feature, "PointXYZRGBA_PFHSignature125");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::PPFRGBSignature>(sub_module_Feature, "PointXYZRGBA_PPFRGBSignature");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::PPFSignature>(sub_module_Feature, "PointXYZRGBA_PPFSignature");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::PointNormal>(sub_module_Feature, "PointXYZRGBA_PointNormal");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::PointXYZL>(sub_module_Feature, "PointXYZRGBA_PointXYZL");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_Feature, "PointXYZRGBA_PointXYZRGBNormal");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::PrincipalCurvatures>(sub_module_Feature, "PointXYZRGBA_PrincipalCurvatures");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::PrincipalRadiiRSD>(sub_module_Feature, "PointXYZRGBA_PrincipalRadiiRSD");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::ReferenceFrame>(sub_module_Feature, "PointXYZRGBA_ReferenceFrame");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::SHOT1344>(sub_module_Feature, "PointXYZRGBA_SHOT1344");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::SHOT352>(sub_module_Feature, "PointXYZRGBA_SHOT352");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::ShapeContext1980>(sub_module_Feature, "PointXYZRGBA_ShapeContext1980");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::UniqueShapeContext1960>(sub_module_Feature, "PointXYZRGBA_UniqueShapeContext1960");
    defineFeaturesFeature<pcl::PointXYZRGBA, pcl::VFHSignature308>(sub_module_Feature, "PointXYZRGBA_VFHSignature308");
    defineFeaturesFeature<pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_Feature, "PointXYZRGBNormal_Boundary");
    defineFeaturesFeature<pcl::PointXYZRGBNormal, pcl::CPPFSignature>(sub_module_Feature, "PointXYZRGBNormal_CPPFSignature");
    defineFeaturesFeature<pcl::PointXYZRGBNormal, pcl::Normal>(sub_module_Feature, "PointXYZRGBNormal_Normal");
    defineFeaturesFeature<pcl::PointXYZRGBNormal, pcl::PFHRGBSignature250>(sub_module_Feature, "PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesFeature<pcl::PointXYZRGBNormal, pcl::PPFRGBSignature>(sub_module_Feature, "PointXYZRGBNormal_PPFRGBSignature");
    defineFeaturesFeature<pcl::PointXYZRGBNormal, pcl::PointNormal>(sub_module_Feature, "PointXYZRGBNormal_PointNormal");
    defineFeaturesFeature<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(sub_module_Feature, "PointXYZRGBNormal_PointXYZRGBNormal");
    py::module sub_module_FeatureFromLabels = sub_module.def_submodule("FeatureFromLabels", "Submodule FeatureFromLabels");
    defineFeaturesFeatureFromLabels<pcl::PointXYZ, pcl::PointXYZL, pcl::GFPFHSignature16>(sub_module_FeatureFromLabels, "PointXYZ_PointXYZL_GFPFHSignature16");
    defineFeaturesFeatureFromLabels<pcl::PointXYZI, pcl::PointXYZL, pcl::GFPFHSignature16>(sub_module_FeatureFromLabels, "PointXYZI_PointXYZL_GFPFHSignature16");
    defineFeaturesFeatureFromLabels<pcl::PointXYZL, pcl::PointXYZL, pcl::GFPFHSignature16>(sub_module_FeatureFromLabels, "PointXYZL_PointXYZL_GFPFHSignature16");
    defineFeaturesFeatureFromLabels<pcl::PointXYZRGBA, pcl::PointXYZL, pcl::GFPFHSignature16>(sub_module_FeatureFromLabels, "PointXYZRGBA_PointXYZL_GFPFHSignature16");
    py::module sub_module_FeatureFromNormals = sub_module.def_submodule("FeatureFromNormals", "Submodule FeatureFromNormals");
    defineFeaturesFeatureFromNormals<pcl::PointNormal, pcl::Normal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointNormal_Normal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointNormal, pcl::Normal, pcl::FPFHSignature33>(sub_module_FeatureFromNormals, "PointNormal_Normal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<pcl::PointNormal, pcl::Normal, pcl::PPFSignature>(sub_module_FeatureFromNormals, "PointNormal_Normal_PPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointNormal, pcl::Normal, pcl::VFHSignature308>(sub_module_FeatureFromNormals, "PointNormal_Normal_VFHSignature308");
    defineFeaturesFeatureFromNormals<pcl::PointNormal, pcl::PointNormal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointNormal_PointNormal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FeatureFromNormals, "PointNormal_PointNormal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature>(sub_module_FeatureFromNormals, "PointNormal_PointNormal_PPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointNormal, pcl::PointNormal, pcl::VFHSignature308>(sub_module_FeatureFromNormals, "PointNormal_PointNormal_VFHSignature308");
    defineFeaturesFeatureFromNormals<pcl::PointNormal, pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointNormal_PointXYZRGBNormal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZ_Normal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZ_Normal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::GRSDSignature21>(sub_module_FeatureFromNormals, "PointXYZ_Normal_GRSDSignature21");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Histogram<90>>(sub_module_FeatureFromNormals, "PointXYZ_Normal_Histogram<90>");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::NormalBasedSignature12>(sub_module_FeatureFromNormals, "PointXYZ_Normal_NormalBasedSignature12");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125>(sub_module_FeatureFromNormals, "PointXYZ_Normal_PFHSignature125");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::PPFSignature>(sub_module_FeatureFromNormals, "PointXYZ_Normal_PPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>(sub_module_FeatureFromNormals, "PointXYZ_Normal_PrincipalCurvatures");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::PrincipalRadiiRSD>(sub_module_FeatureFromNormals, "PointXYZ_Normal_PrincipalRadiiRSD");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame>(sub_module_FeatureFromNormals, "PointXYZ_Normal_ReferenceFrame");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::SHOT1344>(sub_module_FeatureFromNormals, "PointXYZ_Normal_SHOT1344");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>(sub_module_FeatureFromNormals, "PointXYZ_Normal_SHOT352");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980>(sub_module_FeatureFromNormals, "PointXYZ_Normal_ShapeContext1980");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZ_Normal_VFHSignature308");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::PointNormal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZ_PointNormal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZ_PointNormal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::PointNormal, pcl::PPFSignature>(sub_module_FeatureFromNormals, "PointXYZ_PointNormal_PPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZ_PointNormal_VFHSignature308");
    defineFeaturesFeatureFromNormals<pcl::PointXYZ, pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZ_PointXYZRGBNormal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZI_Normal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZI_Normal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::GRSDSignature21>(sub_module_FeatureFromNormals, "PointXYZI_Normal_GRSDSignature21");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::Histogram<90>>(sub_module_FeatureFromNormals, "PointXYZI_Normal_Histogram<90>");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient>(sub_module_FeatureFromNormals, "PointXYZI_Normal_IntensityGradient");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::NormalBasedSignature12>(sub_module_FeatureFromNormals, "PointXYZI_Normal_NormalBasedSignature12");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::PFHSignature125>(sub_module_FeatureFromNormals, "PointXYZI_Normal_PFHSignature125");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::PPFSignature>(sub_module_FeatureFromNormals, "PointXYZI_Normal_PPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::PrincipalCurvatures>(sub_module_FeatureFromNormals, "PointXYZI_Normal_PrincipalCurvatures");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::PrincipalRadiiRSD>(sub_module_FeatureFromNormals, "PointXYZI_Normal_PrincipalRadiiRSD");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::ReferenceFrame>(sub_module_FeatureFromNormals, "PointXYZI_Normal_ReferenceFrame");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::SHOT1344>(sub_module_FeatureFromNormals, "PointXYZI_Normal_SHOT1344");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::SHOT352>(sub_module_FeatureFromNormals, "PointXYZI_Normal_SHOT352");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::ShapeContext1980>(sub_module_FeatureFromNormals, "PointXYZI_Normal_ShapeContext1980");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZI_Normal_VFHSignature308");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::PointNormal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZI_PointNormal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZI_PointNormal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::PointNormal, pcl::PPFSignature>(sub_module_FeatureFromNormals, "PointXYZI_PointNormal_PPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::PointNormal, pcl::VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZI_PointNormal_VFHSignature308");
    defineFeaturesFeatureFromNormals<pcl::PointXYZI, pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZI_PointXYZRGBNormal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_PFHSignature125");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::ReferenceFrame>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_ReferenceFrame");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_SHOT1344");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_SHOT352");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_VFHSignature308");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZRGB_PointNormal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGB, pcl::PointNormal, pcl::VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZRGB_PointNormal_VFHSignature308");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGB_PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_CPPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::GRSDSignature21>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_GRSDSignature21");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Histogram<90>>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_Histogram<90>");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::NormalBasedSignature12>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_NormalBasedSignature12");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHSignature125>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PFHSignature125");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PPFRGBSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::PPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::PrincipalCurvatures>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PrincipalCurvatures");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::PrincipalRadiiRSD>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PrincipalRadiiRSD");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::ReferenceFrame>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_ReferenceFrame");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_SHOT1344");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_SHOT352");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::ShapeContext1980>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_ShapeContext1980");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_VFHSignature308");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::PointNormal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::PointNormal, pcl::CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_CPPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::PointNormal, pcl::FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::PointNormal, pcl::PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_PPFRGBSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::PointNormal, pcl::PPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_PPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::PointNormal, pcl::VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_VFHSignature308");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointXYZRGBNormal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointXYZRGBNormal_CPPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointXYZRGBNormal_PPFRGBSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::Normal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_Normal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::Normal, pcl::CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_Normal_CPPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::Normal, pcl::PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_Normal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::Normal, pcl::PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_Normal_PPFRGBSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::PointNormal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointNormal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::PointNormal, pcl::CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointNormal_CPPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::PointNormal, pcl::PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointNormal_PPFRGBSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointXYZRGBNormal_Boundary");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointXYZRGBNormal_CPPFSignature");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointXYZRGBNormal_PPFRGBSignature");
    defineFeaturesFeatureFunctions(sub_module);
}