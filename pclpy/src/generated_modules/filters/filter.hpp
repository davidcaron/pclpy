
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/filter.h>



template<typename PointT>
void defineFiltersFilter(py::module &m, std::string const & suffix) {
    using Class = Filter<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("filter", py::overload_cast<PointCloud &> (&Class::filter));
    cls.def("get_removed_indices", py::overload_cast<> (&Class::getRemovedIndices));
    cls.def("get_removed_indices", py::overload_cast<PointIndices &> (&Class::getRemovedIndices));
        
}



void defineFiltersFilterClasses(py::module &sub_module) {
    py::module sub_module_Filter = sub_module.def_submodule("Filter", "Submodule Filter");
    defineFiltersFilter<Axis>(sub_module_Filter, "Axis");
    defineFiltersFilter<BRISKSignature512>(sub_module_Filter, "BRISKSignature512");
    defineFiltersFilter<Boundary>(sub_module_Filter, "Boundary");
    defineFiltersFilter<CPPFSignature>(sub_module_Filter, "CPPFSignature");
    defineFiltersFilter<ESFSignature640>(sub_module_Filter, "ESFSignature640");
    defineFiltersFilter<FPFHSignature33>(sub_module_Filter, "FPFHSignature33");
    defineFiltersFilter<GRSDSignature21>(sub_module_Filter, "GRSDSignature21");
    defineFiltersFilter<IntensityGradient>(sub_module_Filter, "IntensityGradient");
    defineFiltersFilter<InterestPoint>(sub_module_Filter, "InterestPoint");
    defineFiltersFilter<Label>(sub_module_Filter, "Label");
    defineFiltersFilter<MomentInvariants>(sub_module_Filter, "MomentInvariants");
    defineFiltersFilter<Narf36>(sub_module_Filter, "Narf36");
    defineFiltersFilter<Normal>(sub_module_Filter, "Normal");
    defineFiltersFilter<NormalBasedSignature12>(sub_module_Filter, "NormalBasedSignature12");
    defineFiltersFilter<PFHRGBSignature250>(sub_module_Filter, "PFHRGBSignature250");
    defineFiltersFilter<PFHSignature125>(sub_module_Filter, "PFHSignature125");
    defineFiltersFilter<PPFRGBSignature>(sub_module_Filter, "PPFRGBSignature");
    defineFiltersFilter<PPFSignature>(sub_module_Filter, "PPFSignature");
    defineFiltersFilter<PointDEM>(sub_module_Filter, "PointDEM");
    defineFiltersFilter<PointNormal>(sub_module_Filter, "PointNormal");
    defineFiltersFilter<PointSurfel>(sub_module_Filter, "PointSurfel");
    defineFiltersFilter<PointUV>(sub_module_Filter, "PointUV");
    defineFiltersFilter<PointWithRange>(sub_module_Filter, "PointWithRange");
    defineFiltersFilter<PointWithScale>(sub_module_Filter, "PointWithScale");
    defineFiltersFilter<PointWithViewpoint>(sub_module_Filter, "PointWithViewpoint");
    defineFiltersFilter<PointXY>(sub_module_Filter, "PointXY");
    defineFiltersFilter<PointXYZ>(sub_module_Filter, "PointXYZ");
    defineFiltersFilter<PointXYZHSV>(sub_module_Filter, "PointXYZHSV");
    defineFiltersFilter<PointXYZI>(sub_module_Filter, "PointXYZI");
    defineFiltersFilter<PointXYZINormal>(sub_module_Filter, "PointXYZINormal");
    defineFiltersFilter<PointXYZL>(sub_module_Filter, "PointXYZL");
    defineFiltersFilter<PointXYZLNormal>(sub_module_Filter, "PointXYZLNormal");
    defineFiltersFilter<PointXYZRGB>(sub_module_Filter, "PointXYZRGB");
    defineFiltersFilter<PointXYZRGBA>(sub_module_Filter, "PointXYZRGBA");
    defineFiltersFilter<PointXYZRGBL>(sub_module_Filter, "PointXYZRGBL");
    defineFiltersFilter<PointXYZRGBNormal>(sub_module_Filter, "PointXYZRGBNormal");
    defineFiltersFilter<PrincipalCurvatures>(sub_module_Filter, "PrincipalCurvatures");
    defineFiltersFilter<PrincipalRadiiRSD>(sub_module_Filter, "PrincipalRadiiRSD");
    defineFiltersFilter<ReferenceFrame>(sub_module_Filter, "ReferenceFrame");
    defineFiltersFilter<SHOT1344>(sub_module_Filter, "SHOT1344");
    defineFiltersFilter<SHOT352>(sub_module_Filter, "SHOT352");
    defineFiltersFilter<ShapeContext1980>(sub_module_Filter, "ShapeContext1980");
    defineFiltersFilter<UniqueShapeContext1960>(sub_module_Filter, "UniqueShapeContext1960");
    defineFiltersFilter<VFHSignature308>(sub_module_Filter, "VFHSignature308");
}