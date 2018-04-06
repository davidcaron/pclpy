
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/rops_estimation.h>



template <typename PointInT, typename PointOutT>
void defineFeaturesROPSEstimation(py::module &m, std::string const & suffix) {
    using Class = ROPSEstimation<PointInT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("number_of_partition_bins", &Class::getNumberOfPartitionBins, &Class::setNumberOfPartitionBins);
    cls.def_property("number_of_rotations", &Class::getNumberOfRotations, &Class::setNumberOfRotations);
    cls.def_property("support_radius", &Class::getSupportRadius, &Class::setSupportRadius);
    cls.def_property("triangles", &Class::getTriangles, &Class::setTriangles);
        
}

void defineFeaturesRopsEstimationClasses(py::module &sub_module) {
    py::module sub_module_ROPSEstimation = sub_module.def_submodule("ROPSEstimation", "Submodule ROPSEstimation");
    defineFeaturesROPSEstimation<PointNormal, Histogram<135>>(sub_module_ROPSEstimation, "PointNormal_Histogram<135>");
    defineFeaturesROPSEstimation<PointXYZ, Histogram<135>>(sub_module_ROPSEstimation, "PointXYZ_Histogram<135>");
    defineFeaturesROPSEstimation<PointXYZI, Histogram<135>>(sub_module_ROPSEstimation, "PointXYZI_Histogram<135>");
    defineFeaturesROPSEstimation<PointXYZRGBA, Histogram<135>>(sub_module_ROPSEstimation, "PointXYZRGBA_Histogram<135>");
}