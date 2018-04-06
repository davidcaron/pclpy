
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/usc.h>



template <typename PointInT, typename PointOutT = pcl::UniqueShapeContext1960, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesUniqueShapeContext(py::module &m, std::string const & suffix) {
    using Class = UniqueShapeContext<PointInT, PointOutT, PointRFT>;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Feature<PointInT,PointOutT>, FeatureWithLocalReferenceFrames<PointInT,PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("minimal_radius", &Class::getMinimalRadius, &Class::setMinimalRadius);
    cls.def_property("point_density_radius", &Class::getPointDensityRadius, &Class::setPointDensityRadius);
    cls.def_property("local_radius", &Class::getLocalRadius, &Class::setLocalRadius);
        
}

void defineFeaturesUscClasses(py::module &sub_module) {
    py::module sub_module_UniqueShapeContext = sub_module.def_submodule("UniqueShapeContext", "Submodule UniqueShapeContext");
    defineFeaturesUniqueShapeContext<PointXYZ, UniqueShapeContext1960, ReferenceFrame>(sub_module_UniqueShapeContext, "PointXYZ_UniqueShapeContext1960_ReferenceFrame");
    defineFeaturesUniqueShapeContext<PointXYZI, UniqueShapeContext1960, ReferenceFrame>(sub_module_UniqueShapeContext, "PointXYZI_UniqueShapeContext1960_ReferenceFrame");
    defineFeaturesUniqueShapeContext<PointXYZRGBA, UniqueShapeContext1960, ReferenceFrame>(sub_module_UniqueShapeContext, "PointXYZRGBA_UniqueShapeContext1960_ReferenceFrame");
}