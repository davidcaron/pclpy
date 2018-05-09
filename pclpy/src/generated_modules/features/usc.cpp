
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/usc.h>



template <typename PointInT, typename PointOutT = pcl::UniqueShapeContext1960, typename PointRFT = pcl::ReferenceFrame>
void defineFeaturesUniqueShapeContext(py::module &m, std::string const & suffix) {
    using Class = pcl::UniqueShapeContext<PointInT, PointOutT, PointRFT>;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, pcl::FeatureWithLocalReferenceFrames<PointInT, PointRFT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setMinimalRadius", &Class::setMinimalRadius, "radius"_a);
    cls.def("setPointDensityRadius", &Class::setPointDensityRadius, "radius"_a);
    cls.def("setLocalRadius", &Class::setLocalRadius, "radius"_a);
    cls.def("getAzimuthBins", &Class::getAzimuthBins);
    cls.def("getElevationBins", &Class::getElevationBins);
    cls.def("getRadiusBins", &Class::getRadiusBins);
    cls.def("getMinimalRadius", &Class::getMinimalRadius);
    cls.def("getPointDensityRadius", &Class::getPointDensityRadius);
    cls.def("getLocalRadius", &Class::getLocalRadius);
        
}

void defineFeaturesUscFunctions(py::module &m) {
}

void defineFeaturesUscClasses(py::module &sub_module) {
    py::module sub_module_UniqueShapeContext = sub_module.def_submodule("UniqueShapeContext", "Submodule UniqueShapeContext");
    defineFeaturesUniqueShapeContext<pcl::PointXYZ, pcl::UniqueShapeContext1960, pcl::ReferenceFrame>(sub_module_UniqueShapeContext, "PointXYZ_UniqueShapeContext1960_ReferenceFrame");
    defineFeaturesUniqueShapeContext<pcl::PointXYZI, pcl::UniqueShapeContext1960, pcl::ReferenceFrame>(sub_module_UniqueShapeContext, "PointXYZI_UniqueShapeContext1960_ReferenceFrame");
    defineFeaturesUniqueShapeContext<pcl::PointXYZRGBA, pcl::UniqueShapeContext1960, pcl::ReferenceFrame>(sub_module_UniqueShapeContext, "PointXYZRGBA_UniqueShapeContext1960_ReferenceFrame");
}