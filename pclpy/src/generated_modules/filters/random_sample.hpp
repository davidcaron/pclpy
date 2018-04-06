
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/random_sample.h>



template<typename PointT>
void defineFiltersRandomSample(py::module &m, std::string const & suffix) {
    using Class = RandomSample<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def_property("sample", &Class::getSample, &Class::setSample);
    cls.def_property("seed", &Class::getSeed, &Class::setSeed);
        
}



void defineFiltersRandomSampleClasses(py::module &sub_module) {
    py::module sub_module_RandomSample = sub_module.def_submodule("RandomSample", "Submodule RandomSample");
    defineFiltersRandomSample<Axis>(sub_module_RandomSample, "Axis");
    defineFiltersRandomSample<BRISKSignature512>(sub_module_RandomSample, "BRISKSignature512");
    defineFiltersRandomSample<Boundary>(sub_module_RandomSample, "Boundary");
    defineFiltersRandomSample<CPPFSignature>(sub_module_RandomSample, "CPPFSignature");
    defineFiltersRandomSample<ESFSignature640>(sub_module_RandomSample, "ESFSignature640");
    defineFiltersRandomSample<FPFHSignature33>(sub_module_RandomSample, "FPFHSignature33");
    defineFiltersRandomSample<GRSDSignature21>(sub_module_RandomSample, "GRSDSignature21");
    defineFiltersRandomSample<IntensityGradient>(sub_module_RandomSample, "IntensityGradient");
    defineFiltersRandomSample<InterestPoint>(sub_module_RandomSample, "InterestPoint");
    defineFiltersRandomSample<Label>(sub_module_RandomSample, "Label");
    defineFiltersRandomSample<MomentInvariants>(sub_module_RandomSample, "MomentInvariants");
    defineFiltersRandomSample<Narf36>(sub_module_RandomSample, "Narf36");
    defineFiltersRandomSample<Normal>(sub_module_RandomSample, "Normal");
    defineFiltersRandomSample<NormalBasedSignature12>(sub_module_RandomSample, "NormalBasedSignature12");
    defineFiltersRandomSample<PFHRGBSignature250>(sub_module_RandomSample, "PFHRGBSignature250");
    defineFiltersRandomSample<PFHSignature125>(sub_module_RandomSample, "PFHSignature125");
    defineFiltersRandomSample<PPFRGBSignature>(sub_module_RandomSample, "PPFRGBSignature");
    defineFiltersRandomSample<PPFSignature>(sub_module_RandomSample, "PPFSignature");
    defineFiltersRandomSample<PointDEM>(sub_module_RandomSample, "PointDEM");
    defineFiltersRandomSample<PointNormal>(sub_module_RandomSample, "PointNormal");
    defineFiltersRandomSample<PointSurfel>(sub_module_RandomSample, "PointSurfel");
    defineFiltersRandomSample<PointUV>(sub_module_RandomSample, "PointUV");
    defineFiltersRandomSample<PointWithRange>(sub_module_RandomSample, "PointWithRange");
    defineFiltersRandomSample<PointWithScale>(sub_module_RandomSample, "PointWithScale");
    defineFiltersRandomSample<PointWithViewpoint>(sub_module_RandomSample, "PointWithViewpoint");
    defineFiltersRandomSample<PointXY>(sub_module_RandomSample, "PointXY");
    defineFiltersRandomSample<PointXYZ>(sub_module_RandomSample, "PointXYZ");
    defineFiltersRandomSample<PointXYZHSV>(sub_module_RandomSample, "PointXYZHSV");
    defineFiltersRandomSample<PointXYZI>(sub_module_RandomSample, "PointXYZI");
    defineFiltersRandomSample<PointXYZINormal>(sub_module_RandomSample, "PointXYZINormal");
    defineFiltersRandomSample<PointXYZL>(sub_module_RandomSample, "PointXYZL");
    defineFiltersRandomSample<PointXYZLNormal>(sub_module_RandomSample, "PointXYZLNormal");
    defineFiltersRandomSample<PointXYZRGB>(sub_module_RandomSample, "PointXYZRGB");
    defineFiltersRandomSample<PointXYZRGBA>(sub_module_RandomSample, "PointXYZRGBA");
    defineFiltersRandomSample<PointXYZRGBL>(sub_module_RandomSample, "PointXYZRGBL");
    defineFiltersRandomSample<PointXYZRGBNormal>(sub_module_RandomSample, "PointXYZRGBNormal");
    defineFiltersRandomSample<PrincipalCurvatures>(sub_module_RandomSample, "PrincipalCurvatures");
    defineFiltersRandomSample<PrincipalRadiiRSD>(sub_module_RandomSample, "PrincipalRadiiRSD");
    defineFiltersRandomSample<ReferenceFrame>(sub_module_RandomSample, "ReferenceFrame");
    defineFiltersRandomSample<SHOT1344>(sub_module_RandomSample, "SHOT1344");
    defineFiltersRandomSample<SHOT352>(sub_module_RandomSample, "SHOT352");
    defineFiltersRandomSample<ShapeContext1980>(sub_module_RandomSample, "ShapeContext1980");
    defineFiltersRandomSample<UniqueShapeContext1960>(sub_module_RandomSample, "UniqueShapeContext1960");
    defineFiltersRandomSample<VFHSignature308>(sub_module_RandomSample, "VFHSignature308");
}