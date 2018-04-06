
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/feature.h>



template <typename PointInT, typename PointRFT>
void defineFeaturesFeatureWithLocalReferenceFrames(py::module &m, std::string const & suffix) {
    using Class = FeatureWithLocalReferenceFrames<PointInT, PointRFT>;
    using PointCloudLRF = Class::PointCloudLRF;
    using PointCloudLRFPtr = Class::PointCloudLRFPtr;
    using PointCloudLRFConstPtr = Class::PointCloudLRFConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("input_reference_frames", &Class::getInputReferenceFrames, &Class::setInputReferenceFrames);
        
}

template <typename PointInT, typename PointOutT>
void defineFeaturesFeature(py::module &m, std::string const & suffix) {
    using Class = Feature<PointInT, PointOutT>;
    using BaseClass = Class::BaseClass;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("search_surface", &Class::getSearchSurface, &Class::setSearchSurface);
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def_property("k_search", &Class::getKSearch, &Class::setKSearch);
    cls.def_property("radius_search", &Class::getRadiusSearch, &Class::setRadiusSearch);
    cls.def("compute", &Class::compute);
        
}

template <typename PointInT, typename PointLT, typename PointOutT>
void defineFeaturesFeatureFromLabels(py::module &m, std::string const & suffix) {
    using Class = FeatureFromLabels<PointInT, PointLT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("input_labels", &Class::getInputLabels, &Class::setInputLabels);
        
}

template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesFeatureFromNormals(py::module &m, std::string const & suffix) {
    using Class = FeatureFromNormals<PointInT, PointNT, PointOutT>;
    using PointCloudN = Class::PointCloudN;
    using PointCloudNPtr = Class::PointCloudNPtr;
    using PointCloudNConstPtr = Class::PointCloudNConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("input_normals", &Class::getInputNormals, &Class::setInputNormals);
        
}

void defineFeaturesFeatureClasses(py::module &sub_module) {
    py::module sub_module_FeatureWithLocalReferenceFrames = sub_module.def_submodule("FeatureWithLocalReferenceFrames", "Submodule FeatureWithLocalReferenceFrames");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZ, Normal>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZ_Normal");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZ, ReferenceFrame>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZ_ReferenceFrame");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZ, UniqueShapeContext1960>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZ_UniqueShapeContext1960");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZI, Normal>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZI_Normal");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZI, ReferenceFrame>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZI_ReferenceFrame");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZI, UniqueShapeContext1960>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZI_UniqueShapeContext1960");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZRGB, Normal>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZRGB_Normal");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZRGB, ReferenceFrame>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZRGB_ReferenceFrame");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZRGBA, Normal>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZRGBA_Normal");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZRGBA, ReferenceFrame>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZRGBA_ReferenceFrame");
    defineFeaturesFeatureWithLocalReferenceFrames<PointXYZRGBA, UniqueShapeContext1960>(sub_module_FeatureWithLocalReferenceFrames, "PointXYZRGBA_UniqueShapeContext1960");
    py::module sub_module_Feature = sub_module.def_submodule("Feature", "Submodule Feature");
    defineFeaturesFeature<PointNormal, Boundary>(sub_module_Feature, "PointNormal_Boundary");
    defineFeaturesFeature<PointNormal, ESFSignature640>(sub_module_Feature, "PointNormal_ESFSignature640");
    defineFeaturesFeature<PointNormal, FPFHSignature33>(sub_module_Feature, "PointNormal_FPFHSignature33");
    defineFeaturesFeature<PointNormal, Histogram<135>>(sub_module_Feature, "PointNormal_Histogram<135>");
    defineFeaturesFeature<PointNormal, Histogram<153>>(sub_module_Feature, "PointNormal_Histogram<153>");
    defineFeaturesFeature<PointNormal, Normal>(sub_module_Feature, "PointNormal_Normal");
    defineFeaturesFeature<PointNormal, PPFSignature>(sub_module_Feature, "PointNormal_PPFSignature");
    defineFeaturesFeature<PointNormal, PointNormal>(sub_module_Feature, "PointNormal_PointNormal");
    defineFeaturesFeature<PointNormal, PointXYZRGBNormal>(sub_module_Feature, "PointNormal_PointXYZRGBNormal");
    defineFeaturesFeature<PointNormal, VFHSignature308>(sub_module_Feature, "PointNormal_VFHSignature308");
    defineFeaturesFeature<PointSurfel, Normal>(sub_module_Feature, "PointSurfel_Normal");
    defineFeaturesFeature<PointSurfel, PointNormal>(sub_module_Feature, "PointSurfel_PointNormal");
    defineFeaturesFeature<PointSurfel, PointXYZRGBNormal>(sub_module_Feature, "PointSurfel_PointXYZRGBNormal");
    defineFeaturesFeature<PointXYZ, Boundary>(sub_module_Feature, "PointXYZ_Boundary");
    defineFeaturesFeature<PointXYZ, ESFSignature640>(sub_module_Feature, "PointXYZ_ESFSignature640");
    defineFeaturesFeature<PointXYZ, FPFHSignature33>(sub_module_Feature, "PointXYZ_FPFHSignature33");
    defineFeaturesFeature<PointXYZ, GFPFHSignature16>(sub_module_Feature, "PointXYZ_GFPFHSignature16");
    defineFeaturesFeature<PointXYZ, GRSDSignature21>(sub_module_Feature, "PointXYZ_GRSDSignature21");
    defineFeaturesFeature<PointXYZ, Histogram<135>>(sub_module_Feature, "PointXYZ_Histogram<135>");
    defineFeaturesFeature<PointXYZ, Histogram<153>>(sub_module_Feature, "PointXYZ_Histogram<153>");
    defineFeaturesFeature<PointXYZ, Histogram<90>>(sub_module_Feature, "PointXYZ_Histogram<90>");
    defineFeaturesFeature<PointXYZ, MomentInvariants>(sub_module_Feature, "PointXYZ_MomentInvariants");
    defineFeaturesFeature<PointXYZ, Normal>(sub_module_Feature, "PointXYZ_Normal");
    defineFeaturesFeature<PointXYZ, NormalBasedSignature12>(sub_module_Feature, "PointXYZ_NormalBasedSignature12");
    defineFeaturesFeature<PointXYZ, PFHSignature125>(sub_module_Feature, "PointXYZ_PFHSignature125");
    defineFeaturesFeature<PointXYZ, PPFSignature>(sub_module_Feature, "PointXYZ_PPFSignature");
    defineFeaturesFeature<PointXYZ, PointNormal>(sub_module_Feature, "PointXYZ_PointNormal");
    defineFeaturesFeature<PointXYZ, PointXYZL>(sub_module_Feature, "PointXYZ_PointXYZL");
    defineFeaturesFeature<PointXYZ, PointXYZRGBNormal>(sub_module_Feature, "PointXYZ_PointXYZRGBNormal");
    defineFeaturesFeature<PointXYZ, PrincipalCurvatures>(sub_module_Feature, "PointXYZ_PrincipalCurvatures");
    defineFeaturesFeature<PointXYZ, PrincipalRadiiRSD>(sub_module_Feature, "PointXYZ_PrincipalRadiiRSD");
    defineFeaturesFeature<PointXYZ, ReferenceFrame>(sub_module_Feature, "PointXYZ_ReferenceFrame");
    defineFeaturesFeature<PointXYZ, SHOT1344>(sub_module_Feature, "PointXYZ_SHOT1344");
    defineFeaturesFeature<PointXYZ, SHOT352>(sub_module_Feature, "PointXYZ_SHOT352");
    defineFeaturesFeature<PointXYZ, ShapeContext1980>(sub_module_Feature, "PointXYZ_ShapeContext1980");
    defineFeaturesFeature<PointXYZ, UniqueShapeContext1960>(sub_module_Feature, "PointXYZ_UniqueShapeContext1960");
    defineFeaturesFeature<PointXYZ, VFHSignature308>(sub_module_Feature, "PointXYZ_VFHSignature308");
    defineFeaturesFeature<PointXYZI, Boundary>(sub_module_Feature, "PointXYZI_Boundary");
    defineFeaturesFeature<PointXYZI, ESFSignature640>(sub_module_Feature, "PointXYZI_ESFSignature640");
    defineFeaturesFeature<PointXYZI, FPFHSignature33>(sub_module_Feature, "PointXYZI_FPFHSignature33");
    defineFeaturesFeature<PointXYZI, GFPFHSignature16>(sub_module_Feature, "PointXYZI_GFPFHSignature16");
    defineFeaturesFeature<PointXYZI, GRSDSignature21>(sub_module_Feature, "PointXYZI_GRSDSignature21");
    defineFeaturesFeature<PointXYZI, Histogram<135>>(sub_module_Feature, "PointXYZI_Histogram<135>");
    defineFeaturesFeature<PointXYZI, Histogram<153>>(sub_module_Feature, "PointXYZI_Histogram<153>");
    defineFeaturesFeature<PointXYZI, Histogram<20>>(sub_module_Feature, "PointXYZI_Histogram<20>");
    defineFeaturesFeature<PointXYZI, Histogram<32>>(sub_module_Feature, "PointXYZI_Histogram<32>");
    defineFeaturesFeature<PointXYZI, Histogram<90>>(sub_module_Feature, "PointXYZI_Histogram<90>");
    defineFeaturesFeature<PointXYZI, IntensityGradient>(sub_module_Feature, "PointXYZI_IntensityGradient");
    defineFeaturesFeature<PointXYZI, MomentInvariants>(sub_module_Feature, "PointXYZI_MomentInvariants");
    defineFeaturesFeature<PointXYZI, Normal>(sub_module_Feature, "PointXYZI_Normal");
    defineFeaturesFeature<PointXYZI, NormalBasedSignature12>(sub_module_Feature, "PointXYZI_NormalBasedSignature12");
    defineFeaturesFeature<PointXYZI, PFHSignature125>(sub_module_Feature, "PointXYZI_PFHSignature125");
    defineFeaturesFeature<PointXYZI, PPFSignature>(sub_module_Feature, "PointXYZI_PPFSignature");
    defineFeaturesFeature<PointXYZI, PointNormal>(sub_module_Feature, "PointXYZI_PointNormal");
    defineFeaturesFeature<PointXYZI, PointXYZL>(sub_module_Feature, "PointXYZI_PointXYZL");
    defineFeaturesFeature<PointXYZI, PointXYZRGBNormal>(sub_module_Feature, "PointXYZI_PointXYZRGBNormal");
    defineFeaturesFeature<PointXYZI, PrincipalCurvatures>(sub_module_Feature, "PointXYZI_PrincipalCurvatures");
    defineFeaturesFeature<PointXYZI, PrincipalRadiiRSD>(sub_module_Feature, "PointXYZI_PrincipalRadiiRSD");
    defineFeaturesFeature<PointXYZI, ReferenceFrame>(sub_module_Feature, "PointXYZI_ReferenceFrame");
    defineFeaturesFeature<PointXYZI, SHOT1344>(sub_module_Feature, "PointXYZI_SHOT1344");
    defineFeaturesFeature<PointXYZI, SHOT352>(sub_module_Feature, "PointXYZI_SHOT352");
    defineFeaturesFeature<PointXYZI, ShapeContext1980>(sub_module_Feature, "PointXYZI_ShapeContext1980");
    defineFeaturesFeature<PointXYZI, UniqueShapeContext1960>(sub_module_Feature, "PointXYZI_UniqueShapeContext1960");
    defineFeaturesFeature<PointXYZI, VFHSignature308>(sub_module_Feature, "PointXYZI_VFHSignature308");
    defineFeaturesFeature<PointXYZL, GFPFHSignature16>(sub_module_Feature, "PointXYZL_GFPFHSignature16");
    defineFeaturesFeature<PointXYZL, PointXYZL>(sub_module_Feature, "PointXYZL_PointXYZL");
    defineFeaturesFeature<PointXYZRGB, FPFHSignature33>(sub_module_Feature, "PointXYZRGB_FPFHSignature33");
    defineFeaturesFeature<PointXYZRGB, Normal>(sub_module_Feature, "PointXYZRGB_Normal");
    defineFeaturesFeature<PointXYZRGB, PFHRGBSignature250>(sub_module_Feature, "PointXYZRGB_PFHRGBSignature250");
    defineFeaturesFeature<PointXYZRGB, PFHSignature125>(sub_module_Feature, "PointXYZRGB_PFHSignature125");
    defineFeaturesFeature<PointXYZRGB, PointNormal>(sub_module_Feature, "PointXYZRGB_PointNormal");
    defineFeaturesFeature<PointXYZRGB, PointXYZRGBNormal>(sub_module_Feature, "PointXYZRGB_PointXYZRGBNormal");
    defineFeaturesFeature<PointXYZRGB, ReferenceFrame>(sub_module_Feature, "PointXYZRGB_ReferenceFrame");
    defineFeaturesFeature<PointXYZRGB, SHOT1344>(sub_module_Feature, "PointXYZRGB_SHOT1344");
    defineFeaturesFeature<PointXYZRGB, SHOT352>(sub_module_Feature, "PointXYZRGB_SHOT352");
    defineFeaturesFeature<PointXYZRGB, VFHSignature308>(sub_module_Feature, "PointXYZRGB_VFHSignature308");
    defineFeaturesFeature<PointXYZRGBA, Boundary>(sub_module_Feature, "PointXYZRGBA_Boundary");
    defineFeaturesFeature<PointXYZRGBA, CPPFSignature>(sub_module_Feature, "PointXYZRGBA_CPPFSignature");
    defineFeaturesFeature<PointXYZRGBA, ESFSignature640>(sub_module_Feature, "PointXYZRGBA_ESFSignature640");
    defineFeaturesFeature<PointXYZRGBA, FPFHSignature33>(sub_module_Feature, "PointXYZRGBA_FPFHSignature33");
    defineFeaturesFeature<PointXYZRGBA, GFPFHSignature16>(sub_module_Feature, "PointXYZRGBA_GFPFHSignature16");
    defineFeaturesFeature<PointXYZRGBA, GRSDSignature21>(sub_module_Feature, "PointXYZRGBA_GRSDSignature21");
    defineFeaturesFeature<PointXYZRGBA, Histogram<135>>(sub_module_Feature, "PointXYZRGBA_Histogram<135>");
    defineFeaturesFeature<PointXYZRGBA, Histogram<153>>(sub_module_Feature, "PointXYZRGBA_Histogram<153>");
    defineFeaturesFeature<PointXYZRGBA, Histogram<90>>(sub_module_Feature, "PointXYZRGBA_Histogram<90>");
    defineFeaturesFeature<PointXYZRGBA, MomentInvariants>(sub_module_Feature, "PointXYZRGBA_MomentInvariants");
    defineFeaturesFeature<PointXYZRGBA, Normal>(sub_module_Feature, "PointXYZRGBA_Normal");
    defineFeaturesFeature<PointXYZRGBA, NormalBasedSignature12>(sub_module_Feature, "PointXYZRGBA_NormalBasedSignature12");
    defineFeaturesFeature<PointXYZRGBA, PFHRGBSignature250>(sub_module_Feature, "PointXYZRGBA_PFHRGBSignature250");
    defineFeaturesFeature<PointXYZRGBA, PFHSignature125>(sub_module_Feature, "PointXYZRGBA_PFHSignature125");
    defineFeaturesFeature<PointXYZRGBA, PPFRGBSignature>(sub_module_Feature, "PointXYZRGBA_PPFRGBSignature");
    defineFeaturesFeature<PointXYZRGBA, PPFSignature>(sub_module_Feature, "PointXYZRGBA_PPFSignature");
    defineFeaturesFeature<PointXYZRGBA, PointNormal>(sub_module_Feature, "PointXYZRGBA_PointNormal");
    defineFeaturesFeature<PointXYZRGBA, PointXYZL>(sub_module_Feature, "PointXYZRGBA_PointXYZL");
    defineFeaturesFeature<PointXYZRGBA, PointXYZRGBNormal>(sub_module_Feature, "PointXYZRGBA_PointXYZRGBNormal");
    defineFeaturesFeature<PointXYZRGBA, PrincipalCurvatures>(sub_module_Feature, "PointXYZRGBA_PrincipalCurvatures");
    defineFeaturesFeature<PointXYZRGBA, PrincipalRadiiRSD>(sub_module_Feature, "PointXYZRGBA_PrincipalRadiiRSD");
    defineFeaturesFeature<PointXYZRGBA, ReferenceFrame>(sub_module_Feature, "PointXYZRGBA_ReferenceFrame");
    defineFeaturesFeature<PointXYZRGBA, SHOT1344>(sub_module_Feature, "PointXYZRGBA_SHOT1344");
    defineFeaturesFeature<PointXYZRGBA, SHOT352>(sub_module_Feature, "PointXYZRGBA_SHOT352");
    defineFeaturesFeature<PointXYZRGBA, ShapeContext1980>(sub_module_Feature, "PointXYZRGBA_ShapeContext1980");
    defineFeaturesFeature<PointXYZRGBA, UniqueShapeContext1960>(sub_module_Feature, "PointXYZRGBA_UniqueShapeContext1960");
    defineFeaturesFeature<PointXYZRGBA, VFHSignature308>(sub_module_Feature, "PointXYZRGBA_VFHSignature308");
    defineFeaturesFeature<PointXYZRGBNormal, Boundary>(sub_module_Feature, "PointXYZRGBNormal_Boundary");
    defineFeaturesFeature<PointXYZRGBNormal, CPPFSignature>(sub_module_Feature, "PointXYZRGBNormal_CPPFSignature");
    defineFeaturesFeature<PointXYZRGBNormal, Normal>(sub_module_Feature, "PointXYZRGBNormal_Normal");
    defineFeaturesFeature<PointXYZRGBNormal, PFHRGBSignature250>(sub_module_Feature, "PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesFeature<PointXYZRGBNormal, PPFRGBSignature>(sub_module_Feature, "PointXYZRGBNormal_PPFRGBSignature");
    defineFeaturesFeature<PointXYZRGBNormal, PointNormal>(sub_module_Feature, "PointXYZRGBNormal_PointNormal");
    defineFeaturesFeature<PointXYZRGBNormal, PointXYZRGBNormal>(sub_module_Feature, "PointXYZRGBNormal_PointXYZRGBNormal");
    py::module sub_module_FeatureFromLabels = sub_module.def_submodule("FeatureFromLabels", "Submodule FeatureFromLabels");
    defineFeaturesFeatureFromLabels<PointXYZ, PointXYZL, GFPFHSignature16>(sub_module_FeatureFromLabels, "PointXYZ_PointXYZL_GFPFHSignature16");
    defineFeaturesFeatureFromLabels<PointXYZI, PointXYZL, GFPFHSignature16>(sub_module_FeatureFromLabels, "PointXYZI_PointXYZL_GFPFHSignature16");
    defineFeaturesFeatureFromLabels<PointXYZL, PointXYZL, GFPFHSignature16>(sub_module_FeatureFromLabels, "PointXYZL_PointXYZL_GFPFHSignature16");
    defineFeaturesFeatureFromLabels<PointXYZRGBA, PointXYZL, GFPFHSignature16>(sub_module_FeatureFromLabels, "PointXYZRGBA_PointXYZL_GFPFHSignature16");
    py::module sub_module_FeatureFromNormals = sub_module.def_submodule("FeatureFromNormals", "Submodule FeatureFromNormals");
    defineFeaturesFeatureFromNormals<PointNormal, Normal, Boundary>(sub_module_FeatureFromNormals, "PointNormal_Normal_Boundary");
    defineFeaturesFeatureFromNormals<PointNormal, Normal, FPFHSignature33>(sub_module_FeatureFromNormals, "PointNormal_Normal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<PointNormal, Normal, PPFSignature>(sub_module_FeatureFromNormals, "PointNormal_Normal_PPFSignature");
    defineFeaturesFeatureFromNormals<PointNormal, Normal, VFHSignature308>(sub_module_FeatureFromNormals, "PointNormal_Normal_VFHSignature308");
    defineFeaturesFeatureFromNormals<PointNormal, PointNormal, Boundary>(sub_module_FeatureFromNormals, "PointNormal_PointNormal_Boundary");
    defineFeaturesFeatureFromNormals<PointNormal, PointNormal, FPFHSignature33>(sub_module_FeatureFromNormals, "PointNormal_PointNormal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<PointNormal, PointNormal, PPFSignature>(sub_module_FeatureFromNormals, "PointNormal_PointNormal_PPFSignature");
    defineFeaturesFeatureFromNormals<PointNormal, PointNormal, VFHSignature308>(sub_module_FeatureFromNormals, "PointNormal_PointNormal_VFHSignature308");
    defineFeaturesFeatureFromNormals<PointNormal, PointXYZRGBNormal, Boundary>(sub_module_FeatureFromNormals, "PointNormal_PointXYZRGBNormal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, Boundary>(sub_module_FeatureFromNormals, "PointXYZ_Normal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZ_Normal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, GRSDSignature21>(sub_module_FeatureFromNormals, "PointXYZ_Normal_GRSDSignature21");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, Histogram<90>>(sub_module_FeatureFromNormals, "PointXYZ_Normal_Histogram<90>");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, NormalBasedSignature12>(sub_module_FeatureFromNormals, "PointXYZ_Normal_NormalBasedSignature12");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, PFHSignature125>(sub_module_FeatureFromNormals, "PointXYZ_Normal_PFHSignature125");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, PPFSignature>(sub_module_FeatureFromNormals, "PointXYZ_Normal_PPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, PrincipalCurvatures>(sub_module_FeatureFromNormals, "PointXYZ_Normal_PrincipalCurvatures");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, PrincipalRadiiRSD>(sub_module_FeatureFromNormals, "PointXYZ_Normal_PrincipalRadiiRSD");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, ReferenceFrame>(sub_module_FeatureFromNormals, "PointXYZ_Normal_ReferenceFrame");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, SHOT1344>(sub_module_FeatureFromNormals, "PointXYZ_Normal_SHOT1344");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, SHOT352>(sub_module_FeatureFromNormals, "PointXYZ_Normal_SHOT352");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, ShapeContext1980>(sub_module_FeatureFromNormals, "PointXYZ_Normal_ShapeContext1980");
    defineFeaturesFeatureFromNormals<PointXYZ, Normal, VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZ_Normal_VFHSignature308");
    defineFeaturesFeatureFromNormals<PointXYZ, PointNormal, Boundary>(sub_module_FeatureFromNormals, "PointXYZ_PointNormal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZ, PointNormal, FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZ_PointNormal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<PointXYZ, PointNormal, PPFSignature>(sub_module_FeatureFromNormals, "PointXYZ_PointNormal_PPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZ, PointNormal, VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZ_PointNormal_VFHSignature308");
    defineFeaturesFeatureFromNormals<PointXYZ, PointXYZRGBNormal, Boundary>(sub_module_FeatureFromNormals, "PointXYZ_PointXYZRGBNormal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, Boundary>(sub_module_FeatureFromNormals, "PointXYZI_Normal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZI_Normal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, GRSDSignature21>(sub_module_FeatureFromNormals, "PointXYZI_Normal_GRSDSignature21");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, Histogram<90>>(sub_module_FeatureFromNormals, "PointXYZI_Normal_Histogram<90>");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, IntensityGradient>(sub_module_FeatureFromNormals, "PointXYZI_Normal_IntensityGradient");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, NormalBasedSignature12>(sub_module_FeatureFromNormals, "PointXYZI_Normal_NormalBasedSignature12");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, PFHSignature125>(sub_module_FeatureFromNormals, "PointXYZI_Normal_PFHSignature125");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, PPFSignature>(sub_module_FeatureFromNormals, "PointXYZI_Normal_PPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, PrincipalCurvatures>(sub_module_FeatureFromNormals, "PointXYZI_Normal_PrincipalCurvatures");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, PrincipalRadiiRSD>(sub_module_FeatureFromNormals, "PointXYZI_Normal_PrincipalRadiiRSD");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, ReferenceFrame>(sub_module_FeatureFromNormals, "PointXYZI_Normal_ReferenceFrame");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, SHOT1344>(sub_module_FeatureFromNormals, "PointXYZI_Normal_SHOT1344");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, SHOT352>(sub_module_FeatureFromNormals, "PointXYZI_Normal_SHOT352");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, ShapeContext1980>(sub_module_FeatureFromNormals, "PointXYZI_Normal_ShapeContext1980");
    defineFeaturesFeatureFromNormals<PointXYZI, Normal, VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZI_Normal_VFHSignature308");
    defineFeaturesFeatureFromNormals<PointXYZI, PointNormal, Boundary>(sub_module_FeatureFromNormals, "PointXYZI_PointNormal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZI, PointNormal, FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZI_PointNormal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<PointXYZI, PointNormal, PPFSignature>(sub_module_FeatureFromNormals, "PointXYZI_PointNormal_PPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZI, PointNormal, VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZI_PointNormal_VFHSignature308");
    defineFeaturesFeatureFromNormals<PointXYZI, PointXYZRGBNormal, Boundary>(sub_module_FeatureFromNormals, "PointXYZI_PointXYZRGBNormal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZRGB, Normal, FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<PointXYZRGB, Normal, PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<PointXYZRGB, Normal, PFHSignature125>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_PFHSignature125");
    defineFeaturesFeatureFromNormals<PointXYZRGB, Normal, ReferenceFrame>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_ReferenceFrame");
    defineFeaturesFeatureFromNormals<PointXYZRGB, Normal, SHOT1344>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_SHOT1344");
    defineFeaturesFeatureFromNormals<PointXYZRGB, Normal, SHOT352>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_SHOT352");
    defineFeaturesFeatureFromNormals<PointXYZRGB, Normal, VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZRGB_Normal_VFHSignature308");
    defineFeaturesFeatureFromNormals<PointXYZRGB, PointNormal, FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZRGB_PointNormal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<PointXYZRGB, PointNormal, VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZRGB_PointNormal_VFHSignature308");
    defineFeaturesFeatureFromNormals<PointXYZRGB, PointXYZRGBNormal, PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGB_PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_CPPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, GRSDSignature21>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_GRSDSignature21");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, Histogram<90>>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_Histogram<90>");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, NormalBasedSignature12>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_NormalBasedSignature12");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, PFHSignature125>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PFHSignature125");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PPFRGBSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, PPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, PrincipalCurvatures>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PrincipalCurvatures");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, PrincipalRadiiRSD>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_PrincipalRadiiRSD");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, ReferenceFrame>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_ReferenceFrame");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, SHOT1344>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_SHOT1344");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, SHOT352>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_SHOT352");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, ShapeContext1980>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_ShapeContext1980");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, Normal, VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZRGBA_Normal_VFHSignature308");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, PointNormal, Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, PointNormal, CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_CPPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, PointNormal, FPFHSignature33>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_FPFHSignature33");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, PointNormal, PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_PPFRGBSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, PointNormal, PPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_PPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, PointNormal, VFHSignature308>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointNormal_VFHSignature308");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, PointXYZRGBNormal, Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointXYZRGBNormal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, PointXYZRGBNormal, CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointXYZRGBNormal_CPPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, PointXYZRGBNormal, PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<PointXYZRGBA, PointXYZRGBNormal, PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBA_PointXYZRGBNormal_PPFRGBSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, Normal, Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_Normal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, Normal, CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_Normal_CPPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, Normal, PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_Normal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, Normal, PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_Normal_PPFRGBSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, PointNormal, Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointNormal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, PointNormal, CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointNormal_CPPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, PointNormal, PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointNormal_PPFRGBSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, PointXYZRGBNormal, Boundary>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointXYZRGBNormal_Boundary");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, PointXYZRGBNormal, CPPFSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointXYZRGBNormal_CPPFSignature");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, PointXYZRGBNormal, PFHRGBSignature250>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesFeatureFromNormals<PointXYZRGBNormal, PointXYZRGBNormal, PPFRGBSignature>(sub_module_FeatureFromNormals, "PointXYZRGBNormal_PointXYZRGBNormal_PPFRGBSignature");
}