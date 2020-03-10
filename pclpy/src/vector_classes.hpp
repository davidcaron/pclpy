
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>
// #include <pcl/visualization/common/common.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <boost/shared_ptr.hpp>

namespace py = pybind11;
using namespace pybind11::literals;

template <typename T>
void bindVector(py::module &m, char *suffix)
{
    py::bind_vector<T, boost::shared_ptr<T>>(m, suffix);
}

template <typename T>
void bindMap(py::module &m, char *suffix)
{
    py::bind_map<T, boost::shared_ptr<T>>(m, suffix);
}

void defineVectorClasses(py::module &m_vector)
{
    bindVector<std::vector<int>>(m_vector, (char *)"Int");
    bindVector<std::vector<float>>(m_vector, (char *)"Float");
    bindVector<std::vector<pcl::PointIndices>>(m_vector, (char *)"PointIndices");
    bindVector<std::vector<pcl::Vertices>>(m_vector, (char *)"Vertices");
    // bindVector<std::vector<pcl::visualization::Camera>>(m_vector, (char *)"Camera");

    //all pcl point types
    bindVector<std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>>(m_vector, (char *)"PointXYZ");
    bindVector<std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>>(m_vector, (char *)"PointXYZI");
    bindVector<std::vector<pcl::PointXYZL, Eigen::aligned_allocator<pcl::PointXYZL>>>(m_vector, (char *)"PointXYZL");
    bindVector<std::vector<pcl::Label, Eigen::aligned_allocator<pcl::Label>>>(m_vector, (char *)"Label");
    bindVector<std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>>>(m_vector, (char *)"PointXYZRGBA");
    bindVector<std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>>(m_vector, (char *)"PointXYZRGB");
    bindVector<std::vector<pcl::PointXYZRGBL, Eigen::aligned_allocator<pcl::PointXYZRGBL>>>(m_vector, (char *)"PointXYZRGBL");
    bindVector<std::vector<pcl::PointXYZHSV, Eigen::aligned_allocator<pcl::PointXYZHSV>>>(m_vector, (char *)"PointXYZHSV");
    bindVector<std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY>>>(m_vector, (char *)"PointXY");
    bindVector<std::vector<pcl::InterestPoint, Eigen::aligned_allocator<pcl::InterestPoint>>>(m_vector, (char *)"InterestPoint");
    bindVector<std::vector<pcl::Axis, Eigen::aligned_allocator<pcl::Axis>>>(m_vector, (char *)"Axis");
    bindVector<std::vector<pcl::Normal, Eigen::aligned_allocator<pcl::Normal>>>(m_vector, (char *)"Normal");
    bindVector<std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal>>>(m_vector, (char *)"PointNormal");
    bindVector<std::vector<pcl::PointXYZRGBNormal, Eigen::aligned_allocator<pcl::PointXYZRGBNormal>>>(m_vector, (char *)"PointXYZRGBNormal");
    bindVector<std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>>>(m_vector, (char *)"PointXYZINormal");
    bindVector<std::vector<pcl::PointXYZLNormal, Eigen::aligned_allocator<pcl::PointXYZLNormal>>>(m_vector, (char *)"PointXYZLNormal");
    bindVector<std::vector<pcl::PointWithRange, Eigen::aligned_allocator<pcl::PointWithRange>>>(m_vector, (char *)"PointWithRange");
    bindVector<std::vector<pcl::PointWithViewpoint, Eigen::aligned_allocator<pcl::PointWithViewpoint>>>(m_vector, (char *)"PointWithViewpoint");
    bindVector<std::vector<pcl::MomentInvariants, Eigen::aligned_allocator<pcl::MomentInvariants>>>(m_vector, (char *)"MomentInvariants");
    bindVector<std::vector<pcl::PrincipalRadiiRSD, Eigen::aligned_allocator<pcl::PrincipalRadiiRSD>>>(m_vector, (char *)"PrincipalRadiiRSD");
    bindVector<std::vector<pcl::Boundary, Eigen::aligned_allocator<pcl::Boundary>>>(m_vector, (char *)"Boundary");
    bindVector<std::vector<pcl::PrincipalCurvatures, Eigen::aligned_allocator<pcl::PrincipalCurvatures>>>(m_vector, (char *)"PrincipalCurvatures");
    bindVector<std::vector<pcl::PFHSignature125, Eigen::aligned_allocator<pcl::PFHSignature125>>>(m_vector, (char *)"PFHSignature125");
    bindVector<std::vector<pcl::PFHRGBSignature250, Eigen::aligned_allocator<pcl::PFHRGBSignature250>>>(m_vector, (char *)"PFHRGBSignature250");
    bindVector<std::vector<pcl::PPFSignature, Eigen::aligned_allocator<pcl::PPFSignature>>>(m_vector, (char *)"PPFSignature");
    bindVector<std::vector<pcl::CPPFSignature, Eigen::aligned_allocator<pcl::CPPFSignature>>>(m_vector, (char *)"CPPFSignature");
    bindVector<std::vector<pcl::PPFRGBSignature, Eigen::aligned_allocator<pcl::PPFRGBSignature>>>(m_vector, (char *)"PPFRGBSignature");
    bindVector<std::vector<pcl::NormalBasedSignature12, Eigen::aligned_allocator<pcl::NormalBasedSignature12>>>(m_vector, (char *)"NormalBasedSignature12");
    bindVector<std::vector<pcl::FPFHSignature33, Eigen::aligned_allocator<pcl::FPFHSignature33>>>(m_vector, (char *)"FPFHSignature33");
    bindVector<std::vector<pcl::VFHSignature308, Eigen::aligned_allocator<pcl::VFHSignature308>>>(m_vector, (char *)"VFHSignature308");
    bindVector<std::vector<pcl::GASDSignature512, Eigen::aligned_allocator<pcl::GASDSignature512>>>(m_vector, (char *)"GASDSignature512");
    bindVector<std::vector<pcl::GASDSignature984, Eigen::aligned_allocator<pcl::GASDSignature984>>>(m_vector, (char *)"GASDSignature984");
    bindVector<std::vector<pcl::GASDSignature7992, Eigen::aligned_allocator<pcl::GASDSignature7992>>>(m_vector, (char *)"GASDSignature7992");
    bindVector<std::vector<pcl::GRSDSignature21, Eigen::aligned_allocator<pcl::GRSDSignature21>>>(m_vector, (char *)"GRSDSignature21");
    bindVector<std::vector<pcl::ESFSignature640, Eigen::aligned_allocator<pcl::ESFSignature640>>>(m_vector, (char *)"ESFSignature640");
    bindVector<std::vector<pcl::BRISKSignature512, Eigen::aligned_allocator<pcl::BRISKSignature512>>>(m_vector, (char *)"BRISKSignature512");
    bindVector<std::vector<pcl::Narf36, Eigen::aligned_allocator<pcl::Narf36>>>(m_vector, (char *)"Narf36");
    bindVector<std::vector<pcl::IntensityGradient, Eigen::aligned_allocator<pcl::IntensityGradient>>>(m_vector, (char *)"IntensityGradient");
    bindVector<std::vector<pcl::PointWithScale, Eigen::aligned_allocator<pcl::PointWithScale>>>(m_vector, (char *)"PointWithScale");
    bindVector<std::vector<pcl::PointSurfel, Eigen::aligned_allocator<pcl::PointSurfel>>>(m_vector, (char *)"PointSurfel");
    bindVector<std::vector<pcl::ShapeContext1980, Eigen::aligned_allocator<pcl::ShapeContext1980>>>(m_vector, (char *)"ShapeContext1980");
    bindVector<std::vector<pcl::UniqueShapeContext1960, Eigen::aligned_allocator<pcl::UniqueShapeContext1960>>>(m_vector, (char *)"UniqueShapeContext1960");
    bindVector<std::vector<pcl::SHOT352, Eigen::aligned_allocator<pcl::SHOT352>>>(m_vector, (char *)"SHOT352");
    bindVector<std::vector<pcl::SHOT1344, Eigen::aligned_allocator<pcl::SHOT1344>>>(m_vector, (char *)"SHOT1344");
    bindVector<std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV>>>(m_vector, (char *)"PointUV");
    bindVector<std::vector<pcl::ReferenceFrame, Eigen::aligned_allocator<pcl::ReferenceFrame>>>(m_vector, (char *)"ReferenceFrame");
    bindVector<std::vector<pcl::PointDEM, Eigen::aligned_allocator<pcl::PointDEM>>>(m_vector, (char *)"PointDEM");
    //    bindVector<std::vector<pcl::GRSDSignature21>>(m_vector, (char *)"GRSDSignature21");  // Linking error

    //Supervoxel
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr>>(m_vector, (char *)"map_uint32t_PointXYZ");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZI>::Ptr>>(m_vector, (char *)"map_uint32t_PointXYZI");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZL>::Ptr>>(m_vector, (char *)"map_uint32t_PointXYZL");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr>>(m_vector, (char *)"map_uint32t_PointXYZRGBA");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr>>(m_vector, (char *)"map_uint32t_PointXYZRGB");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBL>::Ptr>>(m_vector, (char *)"map_uint32t_PointXYZRGBL");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZHSV>::Ptr>>(m_vector, (char *)"map_uint32t_PointXYZHSV");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::InterestPoint>::Ptr>>(m_vector, (char *)"map_uint32t_InterestPoint");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointNormal>::Ptr>>(m_vector, (char *)"map_uint32t_PointNormal");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBNormal>::Ptr>>(m_vector, (char *)"map_uint32t_PointXYZRGBNormal");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZINormal>::Ptr>>(m_vector, (char *)"map_uint32t_PointXYZINormal");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZLNormal>::Ptr>>(m_vector, (char *)"map_uint32t_PointXYZLNormal");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointWithRange>::Ptr>>(m_vector, (char *)"map_uint32t_PointWithRange");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointWithViewpoint>::Ptr>>(m_vector, (char *)"map_uint32t_PointWithViewpoint");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointWithScale>::Ptr>>(m_vector, (char *)"map_uint32t_PointWithScale");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointSurfel>::Ptr>>(m_vector, (char *)"map_uint32t_PointSurfel");
    bindMap<std::map<uint32_t, pcl::Supervoxel<pcl::PointDEM>::Ptr>>(m_vector, (char *)"map_uint32t_PointDEM");
}
