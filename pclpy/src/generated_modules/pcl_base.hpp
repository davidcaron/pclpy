
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/pcl_base.h>



template <typename PointT>
void definePCLBase(py::module &m, std::string const & suffix) {
    using Class = PCLBase<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("input_cloud", &Class::getInputCloud, &Class::setInputCloud);
    // Operators not implemented (operator[]);
    cls.def("set_indices", py::overload_cast<const pcl::IndicesPtr &> (&Class::setIndices));
    cls.def("set_indices", py::overload_cast<const pcl::IndicesConstPtr &> (&Class::setIndices));
    cls.def("set_indices", py::overload_cast<const PointIndicesConstPtr &> (&Class::setIndices));
    cls.def("set_indices", py::overload_cast<size_t, size_t, size_t, size_t> (&Class::setIndices));
    cls.def("get_indices", py::overload_cast<> (&Class::getIndices));
    cls.def("get_indices", py::overload_cast<> (&Class::getIndices, py::const_));
        
}

void definePclBaseClasses(py::module &sub_module) {
    py::module sub_module_PCLBase = sub_module.def_submodule("PCLBase", "Submodule PCLBase");
    definePCLBase<Axis>(sub_module_PCLBase, "Axis");
    definePCLBase<BRISKSignature512>(sub_module_PCLBase, "BRISKSignature512");
    definePCLBase<Boundary>(sub_module_PCLBase, "Boundary");
    definePCLBase<CPPFSignature>(sub_module_PCLBase, "CPPFSignature");
    definePCLBase<ESFSignature640>(sub_module_PCLBase, "ESFSignature640");
    definePCLBase<FPFHSignature33>(sub_module_PCLBase, "FPFHSignature33");
    definePCLBase<GRSDSignature21>(sub_module_PCLBase, "GRSDSignature21");
    definePCLBase<IntensityGradient>(sub_module_PCLBase, "IntensityGradient");
    definePCLBase<InterestPoint>(sub_module_PCLBase, "InterestPoint");
    definePCLBase<Label>(sub_module_PCLBase, "Label");
    definePCLBase<MomentInvariants>(sub_module_PCLBase, "MomentInvariants");
    definePCLBase<Narf36>(sub_module_PCLBase, "Narf36");
    definePCLBase<Normal>(sub_module_PCLBase, "Normal");
    definePCLBase<NormalBasedSignature12>(sub_module_PCLBase, "NormalBasedSignature12");
    definePCLBase<PFHRGBSignature250>(sub_module_PCLBase, "PFHRGBSignature250");
    definePCLBase<PFHSignature125>(sub_module_PCLBase, "PFHSignature125");
    definePCLBase<PPFRGBSignature>(sub_module_PCLBase, "PPFRGBSignature");
    definePCLBase<PPFSignature>(sub_module_PCLBase, "PPFSignature");
    definePCLBase<PointDEM>(sub_module_PCLBase, "PointDEM");
    definePCLBase<PointNormal>(sub_module_PCLBase, "PointNormal");
    definePCLBase<PointSurfel>(sub_module_PCLBase, "PointSurfel");
    definePCLBase<PointUV>(sub_module_PCLBase, "PointUV");
    definePCLBase<PointWithRange>(sub_module_PCLBase, "PointWithRange");
    definePCLBase<PointWithScale>(sub_module_PCLBase, "PointWithScale");
    definePCLBase<PointWithViewpoint>(sub_module_PCLBase, "PointWithViewpoint");
    definePCLBase<PointXY>(sub_module_PCLBase, "PointXY");
    definePCLBase<PointXYZ>(sub_module_PCLBase, "PointXYZ");
    definePCLBase<PointXYZHSV>(sub_module_PCLBase, "PointXYZHSV");
    definePCLBase<PointXYZI>(sub_module_PCLBase, "PointXYZI");
    definePCLBase<PointXYZINormal>(sub_module_PCLBase, "PointXYZINormal");
    definePCLBase<PointXYZL>(sub_module_PCLBase, "PointXYZL");
    definePCLBase<PointXYZLNormal>(sub_module_PCLBase, "PointXYZLNormal");
    definePCLBase<PointXYZRGB>(sub_module_PCLBase, "PointXYZRGB");
    definePCLBase<PointXYZRGBA>(sub_module_PCLBase, "PointXYZRGBA");
    definePCLBase<PointXYZRGBL>(sub_module_PCLBase, "PointXYZRGBL");
    definePCLBase<PointXYZRGBNormal>(sub_module_PCLBase, "PointXYZRGBNormal");
    definePCLBase<PrincipalCurvatures>(sub_module_PCLBase, "PrincipalCurvatures");
    definePCLBase<PrincipalRadiiRSD>(sub_module_PCLBase, "PrincipalRadiiRSD");
    definePCLBase<ReferenceFrame>(sub_module_PCLBase, "ReferenceFrame");
    definePCLBase<SHOT1344>(sub_module_PCLBase, "SHOT1344");
    definePCLBase<SHOT352>(sub_module_PCLBase, "SHOT352");
    definePCLBase<ShapeContext1980>(sub_module_PCLBase, "ShapeContext1980");
    definePCLBase<UniqueShapeContext1960>(sub_module_PCLBase, "UniqueShapeContext1960");
    definePCLBase<VFHSignature308>(sub_module_PCLBase, "VFHSignature308");
}