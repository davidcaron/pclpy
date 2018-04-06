
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/grsd.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesGRSDEstimation(py::module &m, std::string const & suffix) {
    using Class = GRSDEstimation<PointInT, PointNT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("radius_search", &Class::getRadiusSearch, &Class::setRadiusSearch);
        
}

void defineFeaturesGrsdClasses(py::module &sub_module) {
    py::module sub_module_GRSDEstimation = sub_module.def_submodule("GRSDEstimation", "Submodule GRSDEstimation");
    defineFeaturesGRSDEstimation<PointXYZ, Normal, GRSDSignature21>(sub_module_GRSDEstimation, "PointXYZ_Normal_GRSDSignature21");
    defineFeaturesGRSDEstimation<PointXYZI, Normal, GRSDSignature21>(sub_module_GRSDEstimation, "PointXYZI_Normal_GRSDSignature21");
    defineFeaturesGRSDEstimation<PointXYZRGBA, Normal, GRSDSignature21>(sub_module_GRSDEstimation, "PointXYZRGBA_Normal_GRSDSignature21");
}