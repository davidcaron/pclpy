
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/rsd.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesRSDEstimation(py::module &m, std::string const & suffix) {
    using Class = RSDEstimation<PointInT, PointNT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("nr_subdivisions", &Class::getNrSubdivisions, &Class::setNrSubdivisions);
    cls.def_property("plane_radius", &Class::getPlaneRadius, &Class::setPlaneRadius);
    cls.def("set_k_search", &Class::setKSearch);
    cls.def_property("save_histograms", &Class::getSaveHistograms, &Class::setSaveHistograms);
        
}

void defineFeaturesRsdClasses(py::module &sub_module) {
    py::module sub_module_RSDEstimation = sub_module.def_submodule("RSDEstimation", "Submodule RSDEstimation");
    defineFeaturesRSDEstimation<PointXYZ, Normal, PrincipalRadiiRSD>(sub_module_RSDEstimation, "PointXYZ_Normal_PrincipalRadiiRSD");
    defineFeaturesRSDEstimation<PointXYZI, Normal, PrincipalRadiiRSD>(sub_module_RSDEstimation, "PointXYZI_Normal_PrincipalRadiiRSD");
    defineFeaturesRSDEstimation<PointXYZRGBA, Normal, PrincipalRadiiRSD>(sub_module_RSDEstimation, "PointXYZRGBA_Normal_PrincipalRadiiRSD");
}