
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/common/common.h>

#include <boost/shared_ptr.hpp>

namespace py = pybind11;
using namespace pybind11::literals;

template <typename T>
void bindVector(py::module &m, char* suffix) {
    py::bind_vector<T, boost::shared_ptr<T>>(m, suffix);
}

void defineVectorClasses(py::module &m_vector) {
    bindVector<std::vector<int>>(m_vector, "Int");
    bindVector<std::vector<float>>(m_vector, "Float");
    bindVector<std::vector<pcl::PointIndices>>(m_vector, "PointIndices");
    bindVector<std::vector<pcl::visualization::Camera>>(m_vector, "Camera");

    //all pcl point types
    bindVector<std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>>(m_vector, "PointXYZ");
    bindVector<std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>>(m_vector, "PointXYZI");
    bindVector<std::vector<pcl::PointXYZL, Eigen::aligned_allocator<pcl::PointXYZL>>>(m_vector, "PointXYZL");
    bindVector<std::vector<pcl::Label, Eigen::aligned_allocator<pcl::Label>>>(m_vector, "Label");
    bindVector<std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>>>(m_vector, "PointXYZRGBA");
    bindVector<std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>>(m_vector, "PointXYZRGB");
    bindVector<std::vector<pcl::PointXYZRGBL, Eigen::aligned_allocator<pcl::PointXYZRGBL>>>(m_vector, "PointXYZRGBL");
    bindVector<std::vector<pcl::PointXYZHSV, Eigen::aligned_allocator<pcl::PointXYZHSV>>>(m_vector, "PointXYZHSV");
    bindVector<std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY>>>(m_vector, "PointXY");
    bindVector<std::vector<pcl::InterestPoint, Eigen::aligned_allocator<pcl::InterestPoint>>>(m_vector, "InterestPoint");
    bindVector<std::vector<pcl::Axis, Eigen::aligned_allocator<pcl::Axis>>>(m_vector, "Axis");
    bindVector<std::vector<pcl::Normal, Eigen::aligned_allocator<pcl::Normal>>>(m_vector, "Normal");
    bindVector<std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal>>>(m_vector, "PointNormal");
    bindVector<std::vector<pcl::PointXYZRGBNormal, Eigen::aligned_allocator<pcl::PointXYZRGBNormal>>>(m_vector, "PointXYZRGBNormal");
    bindVector<std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>>>(m_vector, "PointXYZINormal");
    bindVector<std::vector<pcl::PointXYZLNormal, Eigen::aligned_allocator<pcl::PointXYZLNormal>>>(m_vector, "PointXYZLNormal");
    bindVector<std::vector<pcl::PointWithRange, Eigen::aligned_allocator<pcl::PointWithRange>>>(m_vector, "PointWithRange");
    bindVector<std::vector<pcl::PointWithViewpoint, Eigen::aligned_allocator<pcl::PointWithViewpoint>>>(m_vector, "PointWithViewpoint");
    bindVector<std::vector<pcl::MomentInvariants, Eigen::aligned_allocator<pcl::MomentInvariants>>>(m_vector, "MomentInvariants");
    bindVector<std::vector<pcl::PrincipalRadiiRSD, Eigen::aligned_allocator<pcl::PrincipalRadiiRSD>>>(m_vector, "PrincipalRadiiRSD");
    bindVector<std::vector<pcl::Boundary, Eigen::aligned_allocator<pcl::Boundary>>>(m_vector, "Boundary");
    bindVector<std::vector<pcl::PrincipalCurvatures, Eigen::aligned_allocator<pcl::PrincipalCurvatures>>>(m_vector, "PrincipalCurvatures");
    bindVector<std::vector<pcl::PFHSignature125, Eigen::aligned_allocator<pcl::PFHSignature125>>>(m_vector, "PFHSignature125");
    bindVector<std::vector<pcl::PFHRGBSignature250, Eigen::aligned_allocator<pcl::PFHRGBSignature250>>>(m_vector, "PFHRGBSignature250");
    bindVector<std::vector<pcl::PPFSignature, Eigen::aligned_allocator<pcl::PPFSignature>>>(m_vector, "PPFSignature");
    bindVector<std::vector<pcl::CPPFSignature, Eigen::aligned_allocator<pcl::CPPFSignature>>>(m_vector, "CPPFSignature");
    bindVector<std::vector<pcl::PPFRGBSignature, Eigen::aligned_allocator<pcl::PPFRGBSignature>>>(m_vector, "PPFRGBSignature");
    bindVector<std::vector<pcl::NormalBasedSignature12, Eigen::aligned_allocator<pcl::NormalBasedSignature12>>>(m_vector, "NormalBasedSignature12");
    bindVector<std::vector<pcl::FPFHSignature33, Eigen::aligned_allocator<pcl::FPFHSignature33>>>(m_vector, "FPFHSignature33");
    bindVector<std::vector<pcl::VFHSignature308, Eigen::aligned_allocator<pcl::VFHSignature308>>>(m_vector, "VFHSignature308");
    bindVector<std::vector<pcl::ESFSignature640, Eigen::aligned_allocator<pcl::ESFSignature640>>>(m_vector, "ESFSignature640");
    bindVector<std::vector<pcl::BRISKSignature512, Eigen::aligned_allocator<pcl::BRISKSignature512>>>(m_vector, "BRISKSignature512");
    bindVector<std::vector<pcl::Narf36, Eigen::aligned_allocator<pcl::Narf36>>>(m_vector, "Narf36");
    bindVector<std::vector<pcl::IntensityGradient, Eigen::aligned_allocator<pcl::IntensityGradient>>>(m_vector, "IntensityGradient");
    bindVector<std::vector<pcl::PointWithScale, Eigen::aligned_allocator<pcl::PointWithScale>>>(m_vector, "PointWithScale");
    bindVector<std::vector<pcl::PointSurfel, Eigen::aligned_allocator<pcl::PointSurfel>>>(m_vector, "PointSurfel");
    bindVector<std::vector<pcl::ShapeContext1980, Eigen::aligned_allocator<pcl::ShapeContext1980>>>(m_vector, "ShapeContext1980");
    bindVector<std::vector<pcl::UniqueShapeContext1960, Eigen::aligned_allocator<pcl::UniqueShapeContext1960>>>(m_vector, "UniqueShapeContext1960");
    bindVector<std::vector<pcl::SHOT352, Eigen::aligned_allocator<pcl::SHOT352>>>(m_vector, "SHOT352");
    bindVector<std::vector<pcl::SHOT1344, Eigen::aligned_allocator<pcl::SHOT1344>>>(m_vector, "SHOT1344");
    bindVector<std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV>>>(m_vector, "PointUV");
    bindVector<std::vector<pcl::ReferenceFrame, Eigen::aligned_allocator<pcl::ReferenceFrame>>>(m_vector, "ReferenceFrame");
    bindVector<std::vector<pcl::PointDEM, Eigen::aligned_allocator<pcl::PointDEM>>>(m_vector, "PointDEM");
//    bindVector<std::vector<pcl::GRSDSignature21>>(m_vector, "GRSDSignature21");  // Linking error
}
