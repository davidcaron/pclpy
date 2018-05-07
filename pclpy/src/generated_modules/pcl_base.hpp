
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/pcl_base.h>



template <typename PointT>
void definePCLBase(py::module &m, std::string const & suffix) {
    using Class = pcl::PCLBase<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointIndicesPtr = Class::PointIndicesPtr;
    using PointIndicesConstPtr = Class::PointIndicesConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    // Operators not implemented (operator[]);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("setIndices", py::overload_cast<const pcl::IndicesPtr &> (&Class::setIndices), "indices"_a);
    cls.def("setIndices", py::overload_cast<const pcl::IndicesConstPtr &> (&Class::setIndices), "indices"_a);
    cls.def("setIndices", py::overload_cast<const PointIndicesConstPtr &> (&Class::setIndices), "indices"_a);
    cls.def("setIndices", py::overload_cast<size_t, size_t, size_t, size_t> (&Class::setIndices), "row_start"_a, "col_start"_a, "nb_rows"_a, "nb_cols"_a);
    cls.def("getInputCloud", &Class::getInputCloud);
    cls.def("getIndices", py::overload_cast<> (&Class::getIndices));
    cls.def("getIndices", py::overload_cast<> (&Class::getIndices, py::const_));
        
}

void definePclBaseFunctions(py::module &m) {
}

void definePclBaseClasses(py::module &sub_module) {
    py::module sub_module_PCLBase = sub_module.def_submodule("PCLBase", "Submodule PCLBase");
    definePCLBase<pcl::Axis>(sub_module_PCLBase, "Axis");
    definePCLBase<pcl::BRISKSignature512>(sub_module_PCLBase, "BRISKSignature512");
    definePCLBase<pcl::Boundary>(sub_module_PCLBase, "Boundary");
    definePCLBase<pcl::CPPFSignature>(sub_module_PCLBase, "CPPFSignature");
    definePCLBase<pcl::ESFSignature640>(sub_module_PCLBase, "ESFSignature640");
    definePCLBase<pcl::FPFHSignature33>(sub_module_PCLBase, "FPFHSignature33");
    definePCLBase<pcl::GRSDSignature21>(sub_module_PCLBase, "GRSDSignature21");
    definePCLBase<pcl::IntensityGradient>(sub_module_PCLBase, "IntensityGradient");
    definePCLBase<pcl::InterestPoint>(sub_module_PCLBase, "InterestPoint");
    definePCLBase<pcl::Label>(sub_module_PCLBase, "Label");
    definePCLBase<pcl::MomentInvariants>(sub_module_PCLBase, "MomentInvariants");
    definePCLBase<pcl::Narf36>(sub_module_PCLBase, "Narf36");
    definePCLBase<pcl::Normal>(sub_module_PCLBase, "Normal");
    definePCLBase<pcl::NormalBasedSignature12>(sub_module_PCLBase, "NormalBasedSignature12");
    definePCLBase<pcl::PFHRGBSignature250>(sub_module_PCLBase, "PFHRGBSignature250");
    definePCLBase<pcl::PFHSignature125>(sub_module_PCLBase, "PFHSignature125");
    definePCLBase<pcl::PPFRGBSignature>(sub_module_PCLBase, "PPFRGBSignature");
    definePCLBase<pcl::PPFSignature>(sub_module_PCLBase, "PPFSignature");
    definePCLBase<pcl::PointDEM>(sub_module_PCLBase, "PointDEM");
    definePCLBase<pcl::PointNormal>(sub_module_PCLBase, "PointNormal");
    definePCLBase<pcl::PointSurfel>(sub_module_PCLBase, "PointSurfel");
    definePCLBase<pcl::PointUV>(sub_module_PCLBase, "PointUV");
    definePCLBase<pcl::PointWithRange>(sub_module_PCLBase, "PointWithRange");
    definePCLBase<pcl::PointWithScale>(sub_module_PCLBase, "PointWithScale");
    definePCLBase<pcl::PointWithViewpoint>(sub_module_PCLBase, "PointWithViewpoint");
    definePCLBase<pcl::PointXY>(sub_module_PCLBase, "PointXY");
    definePCLBase<pcl::PointXYZ>(sub_module_PCLBase, "PointXYZ");
    definePCLBase<pcl::PointXYZHSV>(sub_module_PCLBase, "PointXYZHSV");
    definePCLBase<pcl::PointXYZI>(sub_module_PCLBase, "PointXYZI");
    definePCLBase<pcl::PointXYZINormal>(sub_module_PCLBase, "PointXYZINormal");
    definePCLBase<pcl::PointXYZL>(sub_module_PCLBase, "PointXYZL");
    definePCLBase<pcl::PointXYZLNormal>(sub_module_PCLBase, "PointXYZLNormal");
    definePCLBase<pcl::PointXYZRGB>(sub_module_PCLBase, "PointXYZRGB");
    definePCLBase<pcl::PointXYZRGBA>(sub_module_PCLBase, "PointXYZRGBA");
    definePCLBase<pcl::PointXYZRGBL>(sub_module_PCLBase, "PointXYZRGBL");
    definePCLBase<pcl::PointXYZRGBNormal>(sub_module_PCLBase, "PointXYZRGBNormal");
    definePCLBase<pcl::PrincipalCurvatures>(sub_module_PCLBase, "PrincipalCurvatures");
    definePCLBase<pcl::PrincipalRadiiRSD>(sub_module_PCLBase, "PrincipalRadiiRSD");
    definePCLBase<pcl::ReferenceFrame>(sub_module_PCLBase, "ReferenceFrame");
    definePCLBase<pcl::SHOT1344>(sub_module_PCLBase, "SHOT1344");
    definePCLBase<pcl::SHOT352>(sub_module_PCLBase, "SHOT352");
    definePCLBase<pcl::ShapeContext1980>(sub_module_PCLBase, "ShapeContext1980");
    definePCLBase<pcl::UniqueShapeContext1960>(sub_module_PCLBase, "UniqueShapeContext1960");
    definePCLBase<pcl::VFHSignature308>(sub_module_PCLBase, "VFHSignature308");
}