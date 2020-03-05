
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/Vertices.h>
// #include <pcl/visualization/common/common.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <Eigen/Geometry>

PYBIND11_MAKE_OPAQUE(std::vector<int>);
PYBIND11_MAKE_OPAQUE(std::vector<float>);
PYBIND11_MAKE_OPAQUE(std::vector<pcl::PointIndices>);
PYBIND11_MAKE_OPAQUE(std::vector<pcl::Vertices>);
// PYBIND11_MAKE_OPAQUE(std::vector<pcl::visualization::Camera>);

PYBIND11_MAKE_OPAQUE(Eigen::VectorXf);
PYBIND11_MAKE_OPAQUE(Eigen::Quaternionf);

//all pcl point types
typedef std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> make_opaque_PointXYZ;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXYZ);
typedef std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> make_opaque_PointXYZI;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXYZI);
typedef std::vector<pcl::PointXYZL, Eigen::aligned_allocator<pcl::PointXYZL>> make_opaque_PointXYZL;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXYZL);
typedef std::vector<pcl::Label, Eigen::aligned_allocator<pcl::Label>> make_opaque_Label;
PYBIND11_MAKE_OPAQUE(make_opaque_Label);
typedef std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>> make_opaque_PointXYZRGBA;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXYZRGBA);
typedef std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> make_opaque_PointXYZRGB;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXYZRGB);
typedef std::vector<pcl::PointXYZRGBL, Eigen::aligned_allocator<pcl::PointXYZRGBL>> make_opaque_PointXYZRGBL;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXYZRGBL);
typedef std::vector<pcl::PointXYZHSV, Eigen::aligned_allocator<pcl::PointXYZHSV>> make_opaque_PointXYZHSV;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXYZHSV);
typedef std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY>> make_opaque_PointXY;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXY);
typedef std::vector<pcl::InterestPoint, Eigen::aligned_allocator<pcl::InterestPoint>> make_opaque_InterestPoint;
PYBIND11_MAKE_OPAQUE(make_opaque_InterestPoint);
typedef std::vector<pcl::Axis, Eigen::aligned_allocator<pcl::Axis>> make_opaque_Axis;
PYBIND11_MAKE_OPAQUE(make_opaque_Axis);
typedef std::vector<pcl::Normal, Eigen::aligned_allocator<pcl::Normal>> make_opaque_Normal;
PYBIND11_MAKE_OPAQUE(make_opaque_Normal);
typedef std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal>> make_opaque_PointNormal;
PYBIND11_MAKE_OPAQUE(make_opaque_PointNormal);
typedef std::vector<pcl::PointXYZRGBNormal, Eigen::aligned_allocator<pcl::PointXYZRGBNormal>> make_opaque_PointXYZRGBNormal;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXYZRGBNormal);
typedef std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> make_opaque_PointXYZINormal;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXYZINormal);
typedef std::vector<pcl::PointXYZLNormal, Eigen::aligned_allocator<pcl::PointXYZLNormal>> make_opaque_PointXYZLNormal;
PYBIND11_MAKE_OPAQUE(make_opaque_PointXYZLNormal);
typedef std::vector<pcl::PointWithRange, Eigen::aligned_allocator<pcl::PointWithRange>> make_opaque_PointWithRange;
PYBIND11_MAKE_OPAQUE(make_opaque_PointWithRange);
typedef std::vector<pcl::PointWithViewpoint, Eigen::aligned_allocator<pcl::PointWithViewpoint>> make_opaque_PointWithViewpoint;
PYBIND11_MAKE_OPAQUE(make_opaque_PointWithViewpoint);
typedef std::vector<pcl::MomentInvariants, Eigen::aligned_allocator<pcl::MomentInvariants>> make_opaque_MomentInvariants;
PYBIND11_MAKE_OPAQUE(make_opaque_MomentInvariants);
typedef std::vector<pcl::PrincipalRadiiRSD, Eigen::aligned_allocator<pcl::PrincipalRadiiRSD>> make_opaque_PrincipalRadiiRSD;
PYBIND11_MAKE_OPAQUE(make_opaque_PrincipalRadiiRSD);
typedef std::vector<pcl::Boundary, Eigen::aligned_allocator<pcl::Boundary>> make_opaque_Boundary;
PYBIND11_MAKE_OPAQUE(make_opaque_Boundary);
typedef std::vector<pcl::PrincipalCurvatures, Eigen::aligned_allocator<pcl::PrincipalCurvatures>> make_opaque_PrincipalCurvatures;
PYBIND11_MAKE_OPAQUE(make_opaque_PrincipalCurvatures);
typedef std::vector<pcl::PFHSignature125, Eigen::aligned_allocator<pcl::PFHSignature125>> make_opaque_PFHSignature125;
PYBIND11_MAKE_OPAQUE(make_opaque_PFHSignature125);
typedef std::vector<pcl::PFHRGBSignature250, Eigen::aligned_allocator<pcl::PFHRGBSignature250>> make_opaque_PFHRGBSignature250;
PYBIND11_MAKE_OPAQUE(make_opaque_PFHRGBSignature250);
typedef std::vector<pcl::PPFSignature, Eigen::aligned_allocator<pcl::PPFSignature>> make_opaque_PPFSignature;
PYBIND11_MAKE_OPAQUE(make_opaque_PPFSignature);
typedef std::vector<pcl::CPPFSignature, Eigen::aligned_allocator<pcl::CPPFSignature>> make_opaque_CPPFSignature;
PYBIND11_MAKE_OPAQUE(make_opaque_CPPFSignature);
typedef std::vector<pcl::PPFRGBSignature, Eigen::aligned_allocator<pcl::PPFRGBSignature>> make_opaque_PPFRGBSignature;
PYBIND11_MAKE_OPAQUE(make_opaque_PPFRGBSignature);
typedef std::vector<pcl::NormalBasedSignature12, Eigen::aligned_allocator<pcl::NormalBasedSignature12>> make_opaque_NormalBasedSignature12;
PYBIND11_MAKE_OPAQUE(make_opaque_NormalBasedSignature12);
typedef std::vector<pcl::FPFHSignature33, Eigen::aligned_allocator<pcl::FPFHSignature33>> make_opaque_FPFHSignature33;
PYBIND11_MAKE_OPAQUE(make_opaque_FPFHSignature33);
typedef std::vector<pcl::VFHSignature308, Eigen::aligned_allocator<pcl::VFHSignature308>> make_opaque_VFHSignature308;
PYBIND11_MAKE_OPAQUE(make_opaque_VFHSignature308);
typedef std::vector<pcl::ESFSignature640, Eigen::aligned_allocator<pcl::ESFSignature640>> make_opaque_ESFSignature640;
PYBIND11_MAKE_OPAQUE(make_opaque_ESFSignature640);
typedef std::vector<pcl::BRISKSignature512, Eigen::aligned_allocator<pcl::BRISKSignature512>> make_opaque_BRISKSignature512;
PYBIND11_MAKE_OPAQUE(make_opaque_BRISKSignature512);
typedef std::vector<pcl::Narf36, Eigen::aligned_allocator<pcl::Narf36>> make_opaque_Narf36;
PYBIND11_MAKE_OPAQUE(make_opaque_Narf36);
typedef std::vector<pcl::IntensityGradient, Eigen::aligned_allocator<pcl::IntensityGradient>> make_opaque_IntensityGradient;
PYBIND11_MAKE_OPAQUE(make_opaque_IntensityGradient);
typedef std::vector<pcl::PointWithScale, Eigen::aligned_allocator<pcl::PointWithScale>> make_opaque_PointWithScale;
PYBIND11_MAKE_OPAQUE(make_opaque_PointWithScale);
typedef std::vector<pcl::PointSurfel, Eigen::aligned_allocator<pcl::PointSurfel>> make_opaque_PointSurfel;
PYBIND11_MAKE_OPAQUE(make_opaque_PointSurfel);
typedef std::vector<pcl::ShapeContext1980, Eigen::aligned_allocator<pcl::ShapeContext1980>> make_opaque_ShapeContext1980;
PYBIND11_MAKE_OPAQUE(make_opaque_ShapeContext1980);
typedef std::vector<pcl::UniqueShapeContext1960, Eigen::aligned_allocator<pcl::UniqueShapeContext1960>> make_opaque_UniqueShapeContext1960;
PYBIND11_MAKE_OPAQUE(make_opaque_UniqueShapeContext1960);
typedef std::vector<pcl::SHOT352, Eigen::aligned_allocator<pcl::SHOT352>> make_opaque_SHOT352;
PYBIND11_MAKE_OPAQUE(make_opaque_SHOT352);
typedef std::vector<pcl::SHOT1344, Eigen::aligned_allocator<pcl::SHOT1344>> make_opaque_SHOT1344;
PYBIND11_MAKE_OPAQUE(make_opaque_SHOT1344);
typedef std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV>> make_opaque_PointUV;
PYBIND11_MAKE_OPAQUE(make_opaque_PointUV);
typedef std::vector<pcl::ReferenceFrame, Eigen::aligned_allocator<pcl::ReferenceFrame>> make_opaque_ReferenceFrame;
PYBIND11_MAKE_OPAQUE(make_opaque_ReferenceFrame);
typedef std::vector<pcl::PointDEM, Eigen::aligned_allocator<pcl::PointDEM>> make_opaque_PointDEM;
PYBIND11_MAKE_OPAQUE(make_opaque_PointDEM);
// Linking error
//typedef std::vector<pcl::GRSDSignature21, Eigen::aligned_allocator<pcl::GRSDSignature21>> make_opaque_GRSDSignature21;
//PYBIND11_MAKE_OPAQUE(make_opaque_GRSDSignature21);

//Supervoxel
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr> make_opaque_Supervoxel_PointXYZ;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointXYZ);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZI>::Ptr> make_opaque_Supervoxel_PointXYZI;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointXYZI);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZL>::Ptr> make_opaque_Supervoxel_PointXYZL;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointXYZL);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> make_opaque_Supervoxel_PointXYZRGBA;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointXYZRGBA);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> make_opaque_Supervoxel_PointXYZRGB;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointXYZRGB);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBL>::Ptr> make_opaque_Supervoxel_PointXYZRGBL;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointXYZRGBL);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZHSV>::Ptr> make_opaque_Supervoxel_PointXYZHSV;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointXYZHSV);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::InterestPoint>::Ptr> make_opaque_Supervoxel_InterestPoint;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_InterestPoint);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointNormal>::Ptr> make_opaque_Supervoxel_PointNormal;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointNormal);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBNormal>::Ptr> make_opaque_Supervoxel_PointXYZRGBNormal;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointXYZRGBNormal);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZINormal>::Ptr> make_opaque_Supervoxel_PointXYZINormal;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointXYZINormal);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZLNormal>::Ptr> make_opaque_Supervoxel_PointXYZLNormal;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointXYZLNormal);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointWithRange>::Ptr> make_opaque_Supervoxel_PointWithRange;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointWithRange);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointWithViewpoint>::Ptr> make_opaque_Supervoxel_PointWithViewpoint;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointWithViewpoint);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointWithScale>::Ptr> make_opaque_Supervoxel_PointWithScale;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointWithScale);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointSurfel>::Ptr> make_opaque_Supervoxel_PointSurfel;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointSurfel);
typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointDEM>::Ptr> make_opaque_Supervoxel_PointDEM;
PYBIND11_MAKE_OPAQUE(make_opaque_Supervoxel_PointDEM);