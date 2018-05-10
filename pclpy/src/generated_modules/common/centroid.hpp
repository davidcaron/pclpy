
#include <pcl/common/centroid.h>



template <typename PointT>
void defineCommonCentroidPoint(py::module &m, std::string const & suffix) {
    using Class = pcl::CentroidPoint<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("add", &Class::add, "point"_a);
    cls.def("getSize", &Class::getSize);
    cls.def("get", py::overload_cast<pcl::PointXYZ &> (&Class::get<pcl::PointXYZ>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointXYZI &> (&Class::get<pcl::PointXYZI>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointXYZL &> (&Class::get<pcl::PointXYZL>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::Label &> (&Class::get<pcl::Label>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointXYZRGBA &> (&Class::get<pcl::PointXYZRGBA>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointXYZRGB &> (&Class::get<pcl::PointXYZRGB>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointXYZRGBL &> (&Class::get<pcl::PointXYZRGBL>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointXYZHSV &> (&Class::get<pcl::PointXYZHSV>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointXY &> (&Class::get<pcl::PointXY>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::InterestPoint &> (&Class::get<pcl::InterestPoint>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::Axis &> (&Class::get<pcl::Axis>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::Normal &> (&Class::get<pcl::Normal>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointNormal &> (&Class::get<pcl::PointNormal>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointXYZRGBNormal &> (&Class::get<pcl::PointXYZRGBNormal>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointXYZINormal &> (&Class::get<pcl::PointXYZINormal>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointXYZLNormal &> (&Class::get<pcl::PointXYZLNormal>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointWithRange &> (&Class::get<pcl::PointWithRange>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointWithViewpoint &> (&Class::get<pcl::PointWithViewpoint>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::MomentInvariants &> (&Class::get<pcl::MomentInvariants>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PrincipalRadiiRSD &> (&Class::get<pcl::PrincipalRadiiRSD>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::Boundary &> (&Class::get<pcl::Boundary>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PrincipalCurvatures &> (&Class::get<pcl::PrincipalCurvatures>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PFHSignature125 &> (&Class::get<pcl::PFHSignature125>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PFHRGBSignature250 &> (&Class::get<pcl::PFHRGBSignature250>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PPFSignature &> (&Class::get<pcl::PPFSignature>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::CPPFSignature &> (&Class::get<pcl::CPPFSignature>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PPFRGBSignature &> (&Class::get<pcl::PPFRGBSignature>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::NormalBasedSignature12 &> (&Class::get<pcl::NormalBasedSignature12>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::FPFHSignature33 &> (&Class::get<pcl::FPFHSignature33>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::VFHSignature308 &> (&Class::get<pcl::VFHSignature308>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::GRSDSignature21 &> (&Class::get<pcl::GRSDSignature21>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::ESFSignature640 &> (&Class::get<pcl::ESFSignature640>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::BRISKSignature512 &> (&Class::get<pcl::BRISKSignature512>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::Narf36 &> (&Class::get<pcl::Narf36>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::IntensityGradient &> (&Class::get<pcl::IntensityGradient>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointWithScale &> (&Class::get<pcl::PointWithScale>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointSurfel &> (&Class::get<pcl::PointSurfel>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::ShapeContext1980 &> (&Class::get<pcl::ShapeContext1980>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::UniqueShapeContext1960 &> (&Class::get<pcl::UniqueShapeContext1960>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::SHOT352 &> (&Class::get<pcl::SHOT352>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::SHOT1344 &> (&Class::get<pcl::SHOT1344>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointUV &> (&Class::get<pcl::PointUV>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::ReferenceFrame &> (&Class::get<pcl::ReferenceFrame>, py::const_), "point"_a);
    cls.def("get", py::overload_cast<pcl::PointDEM &> (&Class::get<pcl::PointDEM>, py::const_), "point"_a);
        
}

template<typename PointT, typename Scalar>
void defineCommonNdCentroidFunctor(py::module &m, std::string const & suffix) {
    using Class = pcl::NdCentroidFunctor<PointT, Scalar>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointT, Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(), "p"_a, "centroid"_a);
    // Operators not implemented (operator());
        
}

template <typename PointInT, typename PointOutT>
void defineCommonCentroidFunctions1(py::module &m) {
    m.def("computeCentroid", py::overload_cast<const pcl::PointCloud<PointInT> &, PointOutT &> (&pcl::computeCentroid<PointInT, PointOutT>), "cloud"_a, "centroid"_a);
    m.def("computeCentroid", py::overload_cast<const pcl::PointCloud<PointInT> &, const std::vector<int> &, PointOutT &> (&pcl::computeCentroid<PointInT, PointOutT>), "cloud"_a, "indices"_a, "centroid"_a);
}

template <typename PointT>
void defineCommonCentroidFunctions2(py::module &m) {
    m.def("compute3DCentroid", py::overload_cast<pcl::ConstCloudIterator<PointT> &, Eigen::Vector4f &> (&pcl::compute3DCentroid<PointT>), "cloud_iterator"_a, "centroid"_a);
    m.def("compute3DCentroid", py::overload_cast<pcl::ConstCloudIterator<PointT> &, Eigen::Vector4d &> (&pcl::compute3DCentroid<PointT>), "cloud_iterator"_a, "centroid"_a);
    m.def("compute3DCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Vector4f &> (&pcl::compute3DCentroid<PointT>), "cloud"_a, "centroid"_a);
    m.def("compute3DCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Vector4d &> (&pcl::compute3DCentroid<PointT>), "cloud"_a, "centroid"_a);
    m.def("compute3DCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Vector4f &> (&pcl::compute3DCentroid<PointT>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("compute3DCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Vector4d &> (&pcl::compute3DCentroid<PointT>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("compute3DCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Vector4f &> (&pcl::compute3DCentroid<PointT>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("compute3DCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Vector4d &> (&pcl::compute3DCentroid<PointT>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Vector4f &, Eigen::Matrix3f &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Vector4d &, Eigen::Matrix3d &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Vector4f &, Eigen::Matrix3f &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Vector4d &, Eigen::Matrix3d &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Vector4f &, Eigen::Matrix3f &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Vector4d &, Eigen::Matrix3d &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Matrix3f &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Matrix3d &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Matrix3f &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Matrix3d &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Matrix3f &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Matrix3d &> (&pcl::computeCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrixNormalized", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Vector4f &, Eigen::Matrix3f &> (&pcl::computeCovarianceMatrixNormalized<PointT>), "cloud"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrixNormalized", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Vector4d &, Eigen::Matrix3d &> (&pcl::computeCovarianceMatrixNormalized<PointT>), "cloud"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrixNormalized", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Vector4f &, Eigen::Matrix3f &> (&pcl::computeCovarianceMatrixNormalized<PointT>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrixNormalized", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Vector4d &, Eigen::Matrix3d &> (&pcl::computeCovarianceMatrixNormalized<PointT>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrixNormalized", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Vector4f &, Eigen::Matrix3f &> (&pcl::computeCovarianceMatrixNormalized<PointT>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrixNormalized", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Vector4d &, Eigen::Matrix3d &> (&pcl::computeCovarianceMatrixNormalized<PointT>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeMeanAndCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Matrix3f &, Eigen::Vector4f &> (&pcl::computeMeanAndCovarianceMatrix<PointT>), "cloud"_a, "covariance_matrix"_a, "centroid"_a);
    m.def("computeMeanAndCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Matrix3d &, Eigen::Vector4d &> (&pcl::computeMeanAndCovarianceMatrix<PointT>), "cloud"_a, "covariance_matrix"_a, "centroid"_a);
    m.def("computeMeanAndCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Matrix3f &, Eigen::Vector4f &> (&pcl::computeMeanAndCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "covariance_matrix"_a, "centroid"_a);
    m.def("computeMeanAndCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Matrix3d &, Eigen::Vector4d &> (&pcl::computeMeanAndCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "covariance_matrix"_a, "centroid"_a);
    m.def("computeMeanAndCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Matrix3f &, Eigen::Vector4f &> (&pcl::computeMeanAndCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "covariance_matrix"_a, "centroid"_a);
    m.def("computeMeanAndCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Matrix3d &, Eigen::Vector4d &> (&pcl::computeMeanAndCovarianceMatrix<PointT>), "cloud"_a, "indices"_a, "covariance_matrix"_a, "centroid"_a);
    m.def("computeNDCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::VectorXf &> (&pcl::computeNDCentroid<PointT>), "cloud"_a, "centroid"_a);
    m.def("computeNDCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::VectorXd &> (&pcl::computeNDCentroid<PointT>), "cloud"_a, "centroid"_a);
    m.def("computeNDCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::VectorXf &> (&pcl::computeNDCentroid<PointT>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("computeNDCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::VectorXd &> (&pcl::computeNDCentroid<PointT>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("computeNDCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::VectorXf &> (&pcl::computeNDCentroid<PointT>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("computeNDCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::VectorXd &> (&pcl::computeNDCentroid<PointT>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("demeanPointCloud", py::overload_cast<pcl::ConstCloudIterator<PointT> &, const Eigen::Vector4f &, pcl::PointCloud<PointT> &, int> (&pcl::demeanPointCloud<PointT>), "cloud_iterator"_a, "centroid"_a, "cloud_out"_a, "npts"_a=0);
    m.def("demeanPointCloud", py::overload_cast<pcl::ConstCloudIterator<PointT> &, const Eigen::Vector4d &, pcl::PointCloud<PointT> &, int> (&pcl::demeanPointCloud<PointT>), "cloud_iterator"_a, "centroid"_a, "cloud_out"_a, "npts"_a=0);
    m.def("demeanPointCloud", py::overload_cast<pcl::ConstCloudIterator<PointT> &, const Eigen::Vector4f &, pcl::PointCloud<PointT> &> (&pcl::demeanPointCloud<PointT>), "cloud_iterator"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<pcl::ConstCloudIterator<PointT> &, const Eigen::Vector4d &, pcl::PointCloud<PointT> &> (&pcl::demeanPointCloud<PointT>), "cloud_iterator"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Vector4f &, pcl::PointCloud<PointT> &> (&pcl::demeanPointCloud<PointT>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Vector4d &, pcl::PointCloud<PointT> &> (&pcl::demeanPointCloud<PointT>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Vector4f &, pcl::PointCloud<PointT> &> (&pcl::demeanPointCloud<PointT>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Vector4d &, pcl::PointCloud<PointT> &> (&pcl::demeanPointCloud<PointT>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<pcl::ConstCloudIterator<PointT> &, const Eigen::Vector4f &, Eigen::MatrixXf &, int> (&pcl::demeanPointCloud<PointT>), "cloud_iterator"_a, "centroid"_a, "cloud_out"_a, "npts"_a=0);
    m.def("demeanPointCloud", py::overload_cast<pcl::ConstCloudIterator<PointT> &, const Eigen::Vector4d &, Eigen::MatrixXd &, int> (&pcl::demeanPointCloud<PointT>), "cloud_iterator"_a, "centroid"_a, "cloud_out"_a, "npts"_a=0);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Vector4f &, Eigen::MatrixXf &> (&pcl::demeanPointCloud<PointT>), "cloud_in"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Vector4d &, Eigen::MatrixXd &> (&pcl::demeanPointCloud<PointT>), "cloud_in"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Vector4f &, Eigen::MatrixXf &> (&pcl::demeanPointCloud<PointT>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Vector4d &, Eigen::MatrixXd &> (&pcl::demeanPointCloud<PointT>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Vector4f &, Eigen::MatrixXf &> (&pcl::demeanPointCloud<PointT>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Vector4d &, Eigen::MatrixXd &> (&pcl::demeanPointCloud<PointT>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
}

template <typename PointT, typename Scalar>
void defineCommonCentroidFunctions3(py::module &m) {
    m.def("compute3DCentroid", py::overload_cast<pcl::ConstCloudIterator<PointT> &, Eigen::Matrix<Scalar, 4, 1> &> (&pcl::compute3DCentroid<PointT, Scalar>), "cloud_iterator"_a, "centroid"_a);
    m.def("compute3DCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Matrix<Scalar, 4, 1> &> (&pcl::compute3DCentroid<PointT, Scalar>), "cloud"_a, "centroid"_a);
    m.def("compute3DCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Matrix<Scalar, 4, 1> &> (&pcl::compute3DCentroid<PointT, Scalar>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("compute3DCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Matrix<Scalar, 4, 1> &> (&pcl::compute3DCentroid<PointT, Scalar>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, 3, 3> &> (&pcl::computeCovarianceMatrix<PointT, Scalar>), "cloud"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, 3, 3> &> (&pcl::computeCovarianceMatrix<PointT, Scalar>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, 3, 3> &> (&pcl::computeCovarianceMatrix<PointT, Scalar>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Matrix<Scalar, 3, 3> &> (&pcl::computeCovarianceMatrix<PointT, Scalar>), "cloud"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Matrix<Scalar, 3, 3> &> (&pcl::computeCovarianceMatrix<PointT, Scalar>), "cloud"_a, "indices"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Matrix<Scalar, 3, 3> &> (&pcl::computeCovarianceMatrix<PointT, Scalar>), "cloud"_a, "indices"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrixNormalized", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, 3, 3> &> (&pcl::computeCovarianceMatrixNormalized<PointT, Scalar>), "cloud"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrixNormalized", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, 3, 3> &> (&pcl::computeCovarianceMatrixNormalized<PointT, Scalar>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeCovarianceMatrixNormalized", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, 3, 3> &> (&pcl::computeCovarianceMatrixNormalized<PointT, Scalar>), "cloud"_a, "indices"_a, "centroid"_a, "covariance_matrix"_a);
    m.def("computeMeanAndCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Matrix<Scalar, 3, 3> &, Eigen::Matrix<Scalar, 4, 1> &> (&pcl::computeMeanAndCovarianceMatrix<PointT, Scalar>), "cloud"_a, "covariance_matrix"_a, "centroid"_a);
    m.def("computeMeanAndCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Matrix<Scalar, 3, 3> &, Eigen::Matrix<Scalar, 4, 1> &> (&pcl::computeMeanAndCovarianceMatrix<PointT, Scalar>), "cloud"_a, "indices"_a, "covariance_matrix"_a, "centroid"_a);
    m.def("computeMeanAndCovarianceMatrix", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Matrix<Scalar, 3, 3> &, Eigen::Matrix<Scalar, 4, 1> &> (&pcl::computeMeanAndCovarianceMatrix<PointT, Scalar>), "cloud"_a, "indices"_a, "covariance_matrix"_a, "centroid"_a);
    m.def("computeNDCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &> (&pcl::computeNDCentroid<PointT, Scalar>), "cloud"_a, "centroid"_a);
    m.def("computeNDCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &> (&pcl::computeNDCentroid<PointT, Scalar>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("computeNDCentroid", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &> (&pcl::computeNDCentroid<PointT, Scalar>), "cloud"_a, "indices"_a, "centroid"_a);
    m.def("demeanPointCloud", py::overload_cast<pcl::ConstCloudIterator<PointT> &, const Eigen::Matrix<Scalar, 4, 1> &, pcl::PointCloud<PointT> &, int> (&pcl::demeanPointCloud<PointT, Scalar>), "cloud_iterator"_a, "centroid"_a, "cloud_out"_a, "npts"_a=0);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Matrix<Scalar, 4, 1> &, pcl::PointCloud<PointT> &> (&pcl::demeanPointCloud<PointT, Scalar>), "cloud_in"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Matrix<Scalar, 4, 1> &, pcl::PointCloud<PointT> &> (&pcl::demeanPointCloud<PointT, Scalar>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Matrix<Scalar, 4, 1> &, pcl::PointCloud<PointT> &> (&pcl::demeanPointCloud<PointT, Scalar>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<pcl::ConstCloudIterator<PointT> &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &, int> (&pcl::demeanPointCloud<PointT, Scalar>), "cloud_iterator"_a, "centroid"_a, "cloud_out"_a, "npts"_a=0);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &> (&pcl::demeanPointCloud<PointT, Scalar>), "cloud_in"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const std::vector<int> &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &> (&pcl::demeanPointCloud<PointT, Scalar>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
    m.def("demeanPointCloud", py::overload_cast<const pcl::PointCloud<PointT> &, const pcl::PointIndices &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &> (&pcl::demeanPointCloud<PointT, Scalar>), "cloud_in"_a, "indices"_a, "centroid"_a, "cloud_out"_a);
}

void defineCommonCentroidFunctions(py::module &m) {
    defineCommonCentroidFunctions1<pcl::PointXYZ, pcl::PointXYZ>(m);
    defineCommonCentroidFunctions1<pcl::PointXYZ, pcl::PointXYZRGBA>(m);
    defineCommonCentroidFunctions1<pcl::PointXYZRGBA, pcl::PointXYZ>(m);
    defineCommonCentroidFunctions1<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(m);
    defineCommonCentroidFunctions2<pcl::PointXYZ>(m);
    defineCommonCentroidFunctions2<pcl::PointXYZI>(m);
    defineCommonCentroidFunctions2<pcl::PointXYZL>(m);
    defineCommonCentroidFunctions2<pcl::PointXYZRGBA>(m);
    defineCommonCentroidFunctions2<pcl::PointXYZRGB>(m);
    defineCommonCentroidFunctions2<pcl::PointXYZRGBL>(m);
    defineCommonCentroidFunctions2<pcl::PointXYZHSV>(m);
    defineCommonCentroidFunctions2<pcl::InterestPoint>(m);
    defineCommonCentroidFunctions2<pcl::PointNormal>(m);
    defineCommonCentroidFunctions2<pcl::PointXYZRGBNormal>(m);
    defineCommonCentroidFunctions2<pcl::PointXYZINormal>(m);
    defineCommonCentroidFunctions2<pcl::PointXYZLNormal>(m);
    defineCommonCentroidFunctions2<pcl::PointWithRange>(m);
    defineCommonCentroidFunctions2<pcl::PointWithViewpoint>(m);
    defineCommonCentroidFunctions2<pcl::PointWithScale>(m);
    defineCommonCentroidFunctions2<pcl::PointSurfel>(m);
    defineCommonCentroidFunctions2<pcl::PointDEM>(m);
    defineCommonCentroidFunctions3<pcl::PointXYZ, float>(m);
    defineCommonCentroidFunctions3<pcl::PointXYZI, float>(m);
    defineCommonCentroidFunctions3<pcl::PointXYZL, float>(m);
    defineCommonCentroidFunctions3<pcl::PointXYZRGBA, float>(m);
    defineCommonCentroidFunctions3<pcl::PointXYZRGB, float>(m);
    defineCommonCentroidFunctions3<pcl::PointXYZRGBL, float>(m);
    defineCommonCentroidFunctions3<pcl::PointXYZHSV, float>(m);
    defineCommonCentroidFunctions3<pcl::InterestPoint, float>(m);
    defineCommonCentroidFunctions3<pcl::PointNormal, float>(m);
    defineCommonCentroidFunctions3<pcl::PointXYZRGBNormal, float>(m);
    defineCommonCentroidFunctions3<pcl::PointXYZINormal, float>(m);
    defineCommonCentroidFunctions3<pcl::PointXYZLNormal, float>(m);
    defineCommonCentroidFunctions3<pcl::PointWithRange, float>(m);
    defineCommonCentroidFunctions3<pcl::PointWithViewpoint, float>(m);
    defineCommonCentroidFunctions3<pcl::PointWithScale, float>(m);
    defineCommonCentroidFunctions3<pcl::PointSurfel, float>(m);
    defineCommonCentroidFunctions3<pcl::PointDEM, float>(m);
}

void defineCommonCentroidClasses(py::module &sub_module) {
    defineCommonCentroidFunctions(sub_module);
}