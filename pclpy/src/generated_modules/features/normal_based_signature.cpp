
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/normal_based_signature.h>



template <typename PointT, typename PointNT, typename PointFeature>
void defineFeaturesNormalBasedSignatureEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::NormalBasedSignatureEstimation<PointT, PointNT, PointFeature>;
    using FeatureCloud = Class::FeatureCloud;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FeatureFromNormals<PointT, PointNT, PointFeature>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setN", &Class::setN, "n"_a);
    cls.def("setM", &Class::setM, "m"_a);
    cls.def("setNPrime", &Class::setNPrime, "n_prime"_a);
    cls.def("setMPrime", &Class::setMPrime, "m_prime"_a);
    cls.def("setScale", &Class::setScale, "scale"_a);
    cls.def("getN", &Class::getN);
    cls.def("getM", &Class::getM);
    cls.def("getNPrime", &Class::getNPrime);
    cls.def("getMPrime", &Class::getMPrime);
    cls.def("getScale", &Class::getScale);
        
}

void defineFeaturesNormalBasedSignatureFunctions(py::module &m) {
}

void defineFeaturesNormalBasedSignatureClasses(py::module &sub_module) {
    py::module sub_module_NormalBasedSignatureEstimation = sub_module.def_submodule("NormalBasedSignatureEstimation", "Submodule NormalBasedSignatureEstimation");
    defineFeaturesNormalBasedSignatureEstimation<pcl::PointXYZ, pcl::Normal, pcl::NormalBasedSignature12>(sub_module_NormalBasedSignatureEstimation, "PointXYZ_Normal_NormalBasedSignature12");
    defineFeaturesNormalBasedSignatureEstimation<pcl::PointXYZI, pcl::Normal, pcl::NormalBasedSignature12>(sub_module_NormalBasedSignatureEstimation, "PointXYZI_Normal_NormalBasedSignature12");
    defineFeaturesNormalBasedSignatureEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::NormalBasedSignature12>(sub_module_NormalBasedSignatureEstimation, "PointXYZRGBA_Normal_NormalBasedSignature12");
}