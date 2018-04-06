
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/filter_indices.h>



template<typename PointT>
void defineFiltersFilterIndices(py::module &m, std::string const & suffix) {
    using Class = FilterIndices<PointT>;
    using PointCloud = Class::PointCloud;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("negative", &Class::getNegative, &Class::setNegative);
    cls.def_property("keep_organized", &Class::getKeepOrganized, &Class::setKeepOrganized);
    cls.def("set_user_filter_value", &Class::setUserFilterValue);
    cls.def("filter", py::overload_cast<PointCloud &> (&Class::filter));
    cls.def("filter", py::overload_cast<std::vector<int> &> (&Class::filter));
        
}



void defineFiltersFilterIndicesClasses(py::module &sub_module) {
    py::module sub_module_FilterIndices = sub_module.def_submodule("FilterIndices", "Submodule FilterIndices");
    defineFiltersFilterIndices<Axis>(sub_module_FilterIndices, "Axis");
    defineFiltersFilterIndices<BRISKSignature512>(sub_module_FilterIndices, "BRISKSignature512");
    defineFiltersFilterIndices<Boundary>(sub_module_FilterIndices, "Boundary");
    defineFiltersFilterIndices<CPPFSignature>(sub_module_FilterIndices, "CPPFSignature");
    defineFiltersFilterIndices<ESFSignature640>(sub_module_FilterIndices, "ESFSignature640");
    defineFiltersFilterIndices<FPFHSignature33>(sub_module_FilterIndices, "FPFHSignature33");
    defineFiltersFilterIndices<GRSDSignature21>(sub_module_FilterIndices, "GRSDSignature21");
    defineFiltersFilterIndices<IntensityGradient>(sub_module_FilterIndices, "IntensityGradient");
    defineFiltersFilterIndices<InterestPoint>(sub_module_FilterIndices, "InterestPoint");
    defineFiltersFilterIndices<Label>(sub_module_FilterIndices, "Label");
    defineFiltersFilterIndices<MomentInvariants>(sub_module_FilterIndices, "MomentInvariants");
    defineFiltersFilterIndices<Narf36>(sub_module_FilterIndices, "Narf36");
    defineFiltersFilterIndices<Normal>(sub_module_FilterIndices, "Normal");
    defineFiltersFilterIndices<NormalBasedSignature12>(sub_module_FilterIndices, "NormalBasedSignature12");
    defineFiltersFilterIndices<PFHRGBSignature250>(sub_module_FilterIndices, "PFHRGBSignature250");
    defineFiltersFilterIndices<PFHSignature125>(sub_module_FilterIndices, "PFHSignature125");
    defineFiltersFilterIndices<PPFRGBSignature>(sub_module_FilterIndices, "PPFRGBSignature");
    defineFiltersFilterIndices<PPFSignature>(sub_module_FilterIndices, "PPFSignature");
    defineFiltersFilterIndices<PointDEM>(sub_module_FilterIndices, "PointDEM");
    defineFiltersFilterIndices<PointNormal>(sub_module_FilterIndices, "PointNormal");
    defineFiltersFilterIndices<PointSurfel>(sub_module_FilterIndices, "PointSurfel");
    defineFiltersFilterIndices<PointUV>(sub_module_FilterIndices, "PointUV");
    defineFiltersFilterIndices<PointWithRange>(sub_module_FilterIndices, "PointWithRange");
    defineFiltersFilterIndices<PointWithScale>(sub_module_FilterIndices, "PointWithScale");
    defineFiltersFilterIndices<PointWithViewpoint>(sub_module_FilterIndices, "PointWithViewpoint");
    defineFiltersFilterIndices<PointXY>(sub_module_FilterIndices, "PointXY");
    defineFiltersFilterIndices<PointXYZ>(sub_module_FilterIndices, "PointXYZ");
    defineFiltersFilterIndices<PointXYZHSV>(sub_module_FilterIndices, "PointXYZHSV");
    defineFiltersFilterIndices<PointXYZI>(sub_module_FilterIndices, "PointXYZI");
    defineFiltersFilterIndices<PointXYZINormal>(sub_module_FilterIndices, "PointXYZINormal");
    defineFiltersFilterIndices<PointXYZL>(sub_module_FilterIndices, "PointXYZL");
    defineFiltersFilterIndices<PointXYZLNormal>(sub_module_FilterIndices, "PointXYZLNormal");
    defineFiltersFilterIndices<PointXYZRGB>(sub_module_FilterIndices, "PointXYZRGB");
    defineFiltersFilterIndices<PointXYZRGBA>(sub_module_FilterIndices, "PointXYZRGBA");
    defineFiltersFilterIndices<PointXYZRGBL>(sub_module_FilterIndices, "PointXYZRGBL");
    defineFiltersFilterIndices<PointXYZRGBNormal>(sub_module_FilterIndices, "PointXYZRGBNormal");
    defineFiltersFilterIndices<PrincipalCurvatures>(sub_module_FilterIndices, "PrincipalCurvatures");
    defineFiltersFilterIndices<PrincipalRadiiRSD>(sub_module_FilterIndices, "PrincipalRadiiRSD");
    defineFiltersFilterIndices<ReferenceFrame>(sub_module_FilterIndices, "ReferenceFrame");
    defineFiltersFilterIndices<SHOT1344>(sub_module_FilterIndices, "SHOT1344");
    defineFiltersFilterIndices<SHOT352>(sub_module_FilterIndices, "SHOT352");
    defineFiltersFilterIndices<ShapeContext1980>(sub_module_FilterIndices, "ShapeContext1980");
    defineFiltersFilterIndices<UniqueShapeContext1960>(sub_module_FilterIndices, "UniqueShapeContext1960");
    defineFiltersFilterIndices<VFHSignature308>(sub_module_FilterIndices, "VFHSignature308");
}