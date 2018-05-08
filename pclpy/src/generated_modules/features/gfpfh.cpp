
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/gfpfh.h>



template <typename PointInT, typename PointLT, typename PointOutT>
void defineFeaturesGFPFHEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::GFPFHEstimation<PointInT, PointLT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::FeatureFromLabels<PointInT, PointLT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("emptyLabel", &Class::emptyLabel);
    cls.def("descriptorSize", &Class::descriptorSize);
    cls.def("compute", &Class::compute, "output"_a);
    cls.def("setOctreeLeafSize", &Class::setOctreeLeafSize, "size"_a);
    cls.def("setNumberOfClasses", &Class::setNumberOfClasses, "n"_a);
    cls.def("getOctreeLeafSize", &Class::getOctreeLeafSize);
    cls.def("getNumberOfClasses", &Class::getNumberOfClasses);
        
}

void defineFeaturesGfpfhFunctions(py::module &m) {
}

void defineFeaturesGfpfhClasses(py::module &sub_module) {
    py::module sub_module_GFPFHEstimation = sub_module.def_submodule("GFPFHEstimation", "Submodule GFPFHEstimation");
    defineFeaturesGFPFHEstimation<pcl::PointXYZ, pcl::PointXYZL, pcl::GFPFHSignature16>(sub_module_GFPFHEstimation, "PointXYZ_PointXYZL_GFPFHSignature16");
    defineFeaturesGFPFHEstimation<pcl::PointXYZI, pcl::PointXYZL, pcl::GFPFHSignature16>(sub_module_GFPFHEstimation, "PointXYZI_PointXYZL_GFPFHSignature16");
    defineFeaturesGFPFHEstimation<pcl::PointXYZL, pcl::PointXYZL, pcl::GFPFHSignature16>(sub_module_GFPFHEstimation, "PointXYZL_PointXYZL_GFPFHSignature16");
    defineFeaturesGFPFHEstimation<pcl::PointXYZRGBA, pcl::PointXYZL, pcl::GFPFHSignature16>(sub_module_GFPFHEstimation, "PointXYZRGBA_PointXYZL_GFPFHSignature16");
}