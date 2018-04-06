
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/normal_based_signature.h>



template <typename PointT, typename PointNT, typename PointFeature>
void defineFeaturesNormalBasedSignatureEstimation(py::module &m, std::string const & suffix) {
    using Class = NormalBasedSignatureEstimation<PointT, PointNT, PointFeature>;
    using FeatureCloud = Class::FeatureCloud;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FeatureFromNormals<PointT,PointNT,PointFeature>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("n", &Class::getN, &Class::setN);
    cls.def_property("m", &Class::getM, &Class::setM);
    cls.def_property("n_prime", &Class::getNPrime, &Class::setNPrime);
    cls.def_property("m_prime", &Class::getMPrime, &Class::setMPrime);
    cls.def_property("scale", &Class::getScale, &Class::setScale);
        
}

void defineFeaturesNormalBasedSignatureClasses(py::module &sub_module) {
    py::module sub_module_NormalBasedSignatureEstimation = sub_module.def_submodule("NormalBasedSignatureEstimation", "Submodule NormalBasedSignatureEstimation");
    defineFeaturesNormalBasedSignatureEstimation<PointXYZ, Normal, NormalBasedSignature12>(sub_module_NormalBasedSignatureEstimation, "PointXYZ_Normal_NormalBasedSignature12");
    defineFeaturesNormalBasedSignatureEstimation<PointXYZI, Normal, NormalBasedSignature12>(sub_module_NormalBasedSignatureEstimation, "PointXYZI_Normal_NormalBasedSignature12");
    defineFeaturesNormalBasedSignatureEstimation<PointXYZRGBA, Normal, NormalBasedSignature12>(sub_module_NormalBasedSignatureEstimation, "PointXYZRGBA_Normal_NormalBasedSignature12");
}