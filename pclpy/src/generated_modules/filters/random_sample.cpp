
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/filters/random_sample.h>



template<typename PointT>
void defineFiltersRandomSample(py::module &m, std::string const & suffix) {
    using Class = pcl::RandomSample<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def("setSample", &Class::setSample, "sample"_a);
    cls.def("setSeed", &Class::setSeed, "seed"_a);
    cls.def("getSample", &Class::getSample);
    cls.def("getSeed", &Class::getSeed);
        
}

void defineFiltersRandomSampleFunctions(py::module &m) {
}

void defineFiltersRandomSampleClasses(py::module &sub_module) {
    py::module sub_module_RandomSample = sub_module.def_submodule("RandomSample", "Submodule RandomSample");
    defineFiltersRandomSample<pcl::Axis>(sub_module_RandomSample, "Axis");
    defineFiltersRandomSample<pcl::BRISKSignature512>(sub_module_RandomSample, "BRISKSignature512");
    defineFiltersRandomSample<pcl::Boundary>(sub_module_RandomSample, "Boundary");
    defineFiltersRandomSample<pcl::CPPFSignature>(sub_module_RandomSample, "CPPFSignature");
    defineFiltersRandomSample<pcl::ESFSignature640>(sub_module_RandomSample, "ESFSignature640");
    defineFiltersRandomSample<pcl::FPFHSignature33>(sub_module_RandomSample, "FPFHSignature33");
    defineFiltersRandomSample<pcl::GRSDSignature21>(sub_module_RandomSample, "GRSDSignature21");
    defineFiltersRandomSample<pcl::IntensityGradient>(sub_module_RandomSample, "IntensityGradient");
    defineFiltersRandomSample<pcl::InterestPoint>(sub_module_RandomSample, "InterestPoint");
    defineFiltersRandomSample<pcl::Label>(sub_module_RandomSample, "Label");
    defineFiltersRandomSample<pcl::MomentInvariants>(sub_module_RandomSample, "MomentInvariants");
    defineFiltersRandomSample<pcl::Narf36>(sub_module_RandomSample, "Narf36");
    defineFiltersRandomSample<pcl::Normal>(sub_module_RandomSample, "Normal");
    defineFiltersRandomSample<pcl::NormalBasedSignature12>(sub_module_RandomSample, "NormalBasedSignature12");
    defineFiltersRandomSample<pcl::PFHRGBSignature250>(sub_module_RandomSample, "PFHRGBSignature250");
    defineFiltersRandomSample<pcl::PFHSignature125>(sub_module_RandomSample, "PFHSignature125");
    defineFiltersRandomSample<pcl::PPFRGBSignature>(sub_module_RandomSample, "PPFRGBSignature");
    defineFiltersRandomSample<pcl::PPFSignature>(sub_module_RandomSample, "PPFSignature");
    defineFiltersRandomSample<pcl::PointDEM>(sub_module_RandomSample, "PointDEM");
    defineFiltersRandomSample<pcl::PointNormal>(sub_module_RandomSample, "PointNormal");
    defineFiltersRandomSample<pcl::PointSurfel>(sub_module_RandomSample, "PointSurfel");
    defineFiltersRandomSample<pcl::PointUV>(sub_module_RandomSample, "PointUV");
    defineFiltersRandomSample<pcl::PointWithRange>(sub_module_RandomSample, "PointWithRange");
    defineFiltersRandomSample<pcl::PointWithScale>(sub_module_RandomSample, "PointWithScale");
    defineFiltersRandomSample<pcl::PointWithViewpoint>(sub_module_RandomSample, "PointWithViewpoint");
    defineFiltersRandomSample<pcl::PointXY>(sub_module_RandomSample, "PointXY");
    defineFiltersRandomSample<pcl::PointXYZ>(sub_module_RandomSample, "PointXYZ");
    defineFiltersRandomSample<pcl::PointXYZHSV>(sub_module_RandomSample, "PointXYZHSV");
    defineFiltersRandomSample<pcl::PointXYZI>(sub_module_RandomSample, "PointXYZI");
    defineFiltersRandomSample<pcl::PointXYZINormal>(sub_module_RandomSample, "PointXYZINormal");
    defineFiltersRandomSample<pcl::PointXYZL>(sub_module_RandomSample, "PointXYZL");
    defineFiltersRandomSample<pcl::PointXYZLNormal>(sub_module_RandomSample, "PointXYZLNormal");
    defineFiltersRandomSample<pcl::PointXYZRGB>(sub_module_RandomSample, "PointXYZRGB");
    defineFiltersRandomSample<pcl::PointXYZRGBA>(sub_module_RandomSample, "PointXYZRGBA");
    defineFiltersRandomSample<pcl::PointXYZRGBL>(sub_module_RandomSample, "PointXYZRGBL");
    defineFiltersRandomSample<pcl::PointXYZRGBNormal>(sub_module_RandomSample, "PointXYZRGBNormal");
    defineFiltersRandomSample<pcl::PrincipalCurvatures>(sub_module_RandomSample, "PrincipalCurvatures");
    defineFiltersRandomSample<pcl::PrincipalRadiiRSD>(sub_module_RandomSample, "PrincipalRadiiRSD");
    defineFiltersRandomSample<pcl::ReferenceFrame>(sub_module_RandomSample, "ReferenceFrame");
    defineFiltersRandomSample<pcl::SHOT1344>(sub_module_RandomSample, "SHOT1344");
    defineFiltersRandomSample<pcl::SHOT352>(sub_module_RandomSample, "SHOT352");
    defineFiltersRandomSample<pcl::ShapeContext1980>(sub_module_RandomSample, "ShapeContext1980");
    defineFiltersRandomSample<pcl::UniqueShapeContext1960>(sub_module_RandomSample, "UniqueShapeContext1960");
    defineFiltersRandomSample<pcl::VFHSignature308>(sub_module_RandomSample, "VFHSignature308");
}